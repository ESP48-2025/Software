#include "mbed.h"
#include "Bluetooth.h"
#include "EncoderClass.h"
#include "IR_array.h"
#include "MotorClass.h"
#include "PID.h"
#include <math.h>

// =====================================================
// Hardware objects
// =====================================================

DigitalOut motorEnable(PA_13, 0);

// Motors
// Motor(PWM, Direction, Mode, EncoderA, EncoderB, EncoderSampleFreq)
Motor motorLeft(D15, PC_9, PC_5, PC_11, PD_2, 50.0f);
Motor motorRight(D14, PC_8, PC_6, PC_10, PC_12, 50.0f);

// Bluetooth
Bluetooth ble;

// Line sensors
TCRT IRSensors(PC_2, PC_3, A2, A3, A4, A5, 200.0f);

// IR emitter enable pins through Darlington array
Darlington EnTcrt(D2, D3, D8, D9, PA_15, PA_14);

// =====================================================
// Friend-style PID objects
// =====================================================

//        kp    ki     kd      t      Ts      out_min  out_max  kaw
PID pidL(1.6f, 0.5f, 0.01f, 0.05f, 0.02f, -0.50f, 0.60f, 0.60f);
PID pidR(1.6f, 0.5f, 0.01f, 0.05f, 0.02f, -0.50f, 0.60f, 0.60f);
PID steerPID(0.4f, 0.0f, 0.1f, 0.05f, 0.02f, -0.35f, 0.35f, 0.15f);

// =====================================================
// Control parameters
// =====================================================

const float CONTROL_DT = 0.01f;     // 10 ms
const float BASE_SPEED = 0.10f;     // m/s
const float KF = 0.40f;             // feedforward

// =====================================================
// U-turn parameters
// =====================================================

// This controls the actual U-turn angle.
// Increase this if it turns less than 180 degrees.
// Decrease this if it turns more than 180 degrees.
const int TURN_PULSES = 2800;

// This is only a safety timeout.
// It does NOT control the turn angle in normal operation.
const int TURN_TIMEOUT_MS = 5000;

// =====================================================
// State variables
// =====================================================

Ticker controlTicker;

volatile bool isRunning = false;
volatile bool inTurn = false;
volatile bool lineLostFlag = false;

float filtL = 0.0f;
float filtR = 0.0f;

float currentTarget = 0.0f;

volatile char bt_cmd = 0;
bool canTurn = true;
Timer turnTimer;

// =====================================================
// Helper functions
// =====================================================

float clampf(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

void stopMotors()
{
    motorLeft.setSpeed(0.0f);
    motorRight.setSpeed(0.0f);
}

// =====================================================
// Reset controllers
// =====================================================

void resetControllers()
{
    pidL.reset();
    pidR.reset();
    steerPID.reset();

    filtL = 0.0f;
    filtR = 0.0f;

    currentTarget = 0.0f;
}

// =====================================================
// Main control loop
// Same structure as your friend's Buggy::controlLoop()
// =====================================================

void controlLoop()
{
    if (!isRunning || inTurn) {
        return;
    }

    // -------------------------------------------------
    // 1. Read wheel speeds
    // -------------------------------------------------
    float speedL = -motorLeft.readSpeedMs();
    float speedR =  motorRight.readSpeedMs();

    // -------------------------------------------------
    // 2. Adaptive low-pass filtering
    // -------------------------------------------------
    float alphaL = (fabs(filtL) < 0.06f) ? 0.30f : 0.15f;
    float alphaR = (fabs(filtR) < 0.06f) ? 0.30f : 0.15f;

    filtL = alphaL * speedL + (1.0f - alphaL) * filtL;
    filtR = alphaR * speedR + (1.0f - alphaR) * filtR;

    // -------------------------------------------------
    // 3. Line lost protection
    // -------------------------------------------------
    if (IRSensors.isLineLost()) {
        stopMotors();
        lineLostFlag = true;
        return;
    }

    // -------------------------------------------------
    // 4. Line position and steering PID
    // -------------------------------------------------
    float position = IRSensors.getPosition();

    float steer = steerPID.update(0.0f, position);

    // -------------------------------------------------
    // 5. Smooth target ramp
    // -------------------------------------------------
    if (currentTarget < BASE_SPEED) {
        currentTarget += 0.001f;

        if (currentTarget > BASE_SPEED) {
            currentTarget = BASE_SPEED;
        }
    }

    // -------------------------------------------------
    // 6. Target mixing
    // -------------------------------------------------
    float targetL = currentTarget + steer;
    float targetR = currentTarget - steer;

    // -------------------------------------------------
    // 7. Inner speed PID
    // -------------------------------------------------
    float ctrlL = pidL.update(targetL, filtL);
    float ctrlR = pidR.update(targetR, filtR);

    // -------------------------------------------------
    // 8. Feedforward + PID correction
    // -------------------------------------------------
    float pwmL = KF * targetL + ctrlL;
    float pwmR = KF * targetR + ctrlR;

    // -------------------------------------------------
    // 9. Output clamp
    // -------------------------------------------------
    if (pwmL > 0.95f) pwmL = 0.95f;
    if (pwmL < 0.05f) pwmL = 0.05f;

    if (pwmR > 0.95f) pwmR = 0.95f;
    if (pwmR < 0.05f) pwmR = 0.05f;

    motorLeft.setSpeed(pwmL);
    motorRight.setSpeed(pwmR);
}

// =====================================================
// Start / stop
// =====================================================

void startBuggy()
{
    controlTicker.detach();

    motorLeft.begin();
    motorRight.begin();
    IRSensors.start();

    wait_ms(150);

    resetControllers();

    lineLostFlag = false;
    isRunning = true;
    inTurn = false;

    controlTicker.attach(&controlLoop, CONTROL_DT);
}

void stopBuggy()
{
    isRunning = false;
    inTurn = false;

    controlTicker.detach();

    stopMotors();

    pidL.reset();
    pidR.reset();
    steerPID.reset();

    currentTarget = 0.0f;
}

// =====================================================
// Safe update
// Same idea as friend's Buggy::update()
// =====================================================

void updateBuggy()
{
    if (lineLostFlag && isRunning && !inTurn) {
        stopBuggy();
    }
}

// =====================================================
// U-turn
// Corrected for your global main.cpp structure
// =====================================================

void turnaround()
{
    if (inTurn) {
        return;
    }

    inTurn = true;
    isRunning = false;

    controlTicker.detach();

    motorLeft.setSpeed(0.0f);
    motorRight.setSpeed(0.0f);
    wait_ms(150);

    int startL = motorLeft.getPulses();
    int startR = motorRight.getPulses();

    Timer timer;
    timer.start();

    // Pivot turn:
    // left wheel forward, right wheel reverse
    motorLeft.setDirection(1);
    motorRight.setDirection(0);

    motorLeft.setSpeed(0.30f);
    motorRight.setSpeed(0.30f);

    while (timer.read_ms() < TURN_TIMEOUT_MS) {

        int deltaL = motorLeft.getPulses() - startL;
        int deltaR = motorRight.getPulses() - startR;

        if (deltaL < 0) deltaL = -deltaL;
        if (deltaR < 0) deltaR = -deltaR;

        int avgDelta = (deltaL + deltaR) / 2;

        if (avgDelta >= TURN_PULSES) {
            break;
        }

        wait_ms(5);
    }

    motorLeft.setSpeed(0.0f);
    motorRight.setSpeed(0.0f);

    wait_ms(150);

    // Restore normal forward direction
    motorLeft.setDirection(1);
    motorRight.setDirection(1);

    pidL.reset();
    pidR.reset();
    steerPID.reset();

    filtL = 0.0f;
    filtR = 0.0f;
    currentTarget = 0.0f;

    lineLostFlag = false;
    inTurn = false;
    isRunning = true;

    controlTicker.attach(&controlLoop, CONTROL_DT);
}

// =====================================================
// Main
// =====================================================

int main()
{
    // -------------------------------------------------
    // IR emitter enable
    // -------------------------------------------------
    EnTcrt.writeAll(1, 1, 1, 1, 1, 1);

    // -------------------------------------------------
    // Bluetooth
    // -------------------------------------------------
    ble.begin();

    // -------------------------------------------------
    // Motor driver setup
    // -------------------------------------------------
    motorEnable = 0;

    // Unipolar mode
    motorLeft.bipolar_mode(0);
    motorRight.bipolar_mode(0);

    motorLeft.pwm_period(50);
    motorRight.pwm_period(50);

    motorLeft.setDirection(1);
    motorRight.setDirection(1);

    stopMotors();

    wait_ms(300);

    // -------------------------------------------------
    // Sensor calibration
    // -------------------------------------------------
    IRSensors.calibrate();

    // -------------------------------------------------
    // Enable motor driver and start buggy
    // -------------------------------------------------
    motorEnable = 1;

    startBuggy();

    turnTimer.start();

    while (1) {

        // -------------------------------------------------
        // Normal control update
        // -------------------------------------------------
        updateBuggy();

        // -------------------------------------------------
        // Line lost auto stop
        // -------------------------------------------------
        if (lineLostFlag) {
            stopBuggy();
        }

        // -------------------------------------------------
        // Bluetooth command polling
        // -------------------------------------------------
        if (ble.available()) {
            bt_cmd = ble.readChar();
        }

        // -------------------------------------------------
        // Bluetooth U-turn command
        // -------------------------------------------------
        if (bt_cmd == 't' && canTurn) {

            bt_cmd = 0;
            canTurn = false;

            stopBuggy();
            wait_ms(150);

            turnaround();

            turnTimer.reset();
        }

        // -------------------------------------------------
        // Turn cooldown
        // -------------------------------------------------
        if (!canTurn && turnTimer.read_ms() > 800) {
            canTurn = true;
        }

        // -------------------------------------------------
        // Optional Bluetooth debug
        // -------------------------------------------------
        if (ble.sendAvailable()) {
            ble.sendSpeed(motorLeft.readSpeedMs(), motorRight.readSpeedMs());
        }

        wait_ms(5);
    }
}
