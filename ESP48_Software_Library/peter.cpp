#include "mbed.h"
#include "Bluetooth.h"
#include "Encoder.h"
#include "Motor.h"
#include "IR_array.h"
#include "PID.h"

// ==========================
// Hardware objects
// ==========================
DigitalOut motorEnable(PA_13, 1);

Motor motorLeft(D15, PC_9, PC_5);
Motor motorRight(D14, PC_8, PC_6);

Bluetooth ble;
Encoder encR(PC_10, PC_12, 50.0f);
Encoder encL(PC_11, PD_2, 50.0f);

TCRT IRSensors(PC_2, PC_3, A2, A3, A4, A5, 200.0f);
Darlington EnTcrt(D2, D3, D8, D9, PA_15, PA_14);

// ==========================
// Parameters
// ==========================

// bipolar mode: 0.5 = stop, 1.0 = max forward, 0.0 = max reverse
const float PWM_CENTER = 0.5f;

// forward cruising amount above center
const float BASE_OFFSET = 0.14f;

// compensate natural left/right motor mismatch
// if buggy tends to drift right, slightly increase left side
// if buggy tends to drift left, reduce this or make right larger instead
const float LEFT_BIAS = 0.03f;
const float RIGHT_BIAS = 0.00f;

// line-following gains
const float KP_LINE = 0.06f;
const float KI_LINE = 0.015f;

// max steering correction around center
const float MAX_STEER = 0.12f;

// integral protection
const float I_LIMIT = 1.2f;

// if line is lost, search using last known direction
const float SEARCH_OFFSET = 0.08f;

// if your sensor output is opposite, change this to false
const bool WHITE_IS_HIGH = true;

// minimum total line response to regard line as found
const float LINE_DETECT_THRESHOLD = 0.35f;

// control loop dt, matched to wait_ms(5)
const float DT = 0.005f;

// only integrate when error is not too large
const float I_ACTIVE_ERROR_LIMIT = 0.9f;

// filtering
const float ERROR_FILTER_ALPHA = 0.20f;   // larger => follows new reading faster
const float I_LEAK = 0.995f;              // slight decay each loop

// ==========================
// Helper functions
// ==========================
float clampf(float x, float low, float high) {
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

float processSensor(float x) {
    x = clampf(x, 0.0f, 1.0f);

    if (WHITE_IS_HIGH) {
        return x;
    } else {
        return 1.0f - x;
    }
}

void setMotorBipolar(float leftCmd, float rightCmd) {
    leftCmd  = clampf(leftCmd,  0.0f, 1.0f);
    rightCmd = clampf(rightCmd, 0.0f, 1.0f);

    motorLeft.speed(leftCmd);
    motorRight.speed(rightCmd);
}

void stopMotors() {
    motorLeft.speed(PWM_CENTER);
    motorRight.speed(PWM_CENTER);
}

// weighted line position:
// leftmost = -2.5 ... rightmost = +2.5
float getLineError(float &strength) {
    float s[6];

    for (int i = 0; i < 6; i++) {
        s[i] = processSensor(IRSensors.getCurrentSampleNorm(i));
    }

    const float w[6] = {-2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f};

    float total = 0.0f;
    float weightedSum = 0.0f;

    for (int i = 0; i < 6; i++) {
        total += s[i];
        weightedSum += s[i] * w[i];
    }

    strength = total;

    if (total < 1e-6f) {
        return 0.0f;
    }

    return weightedSum / total;
}

// ==========================
// Main
// ==========================
int main() {
    ble.begin();

    motorEnable = 1;

    motorLeft.bipolar_mode(1);
    motorRight.bipolar_mode(1);

    motorLeft.pwm_period(50);   // 20 kHz
    motorRight.pwm_period(50);  // 20 kHz

    // dir pin may be ignored in bipolar mode on your board,
    // but keep both set consistently
    motorLeft.is_forward(0);
    motorRight.is_forward(0);

    // Turn on all IR emitters
    EnTcrt.writeAll(1, 1, 1, 1, 1, 1);

    stopMotors();
    wait_ms(500);

    ble.sendWords("TD3 white line follow PI start");

    float lastError = 0.0f;
    float filteredError = 0.0f;
    float errorIntegral = 0.0f;

    while (1) {
        IRSensors.update_sample();

        float lineStrength = 0.0f;
        float error = getLineError(lineStrength);

        // low-pass filter on error
        filteredError = (1.0f - ERROR_FILTER_ALPHA) * filteredError
                      + ERROR_FILTER_ALPHA * error;

        // Uncomment this for Bluetooth sensor debugging
        /*
        if (ble.sendAvailable()) {
            ble.sendTCRT(
                IRSensors.getCurrentSampleNorm(0),
                IRSensors.getCurrentSampleNorm(1),
                IRSensors.getCurrentSampleNorm(2),
                IRSensors.getCurrentSampleNorm(3),
                IRSensors.getCurrentSampleNorm(4),
                IRSensors.getCurrentSampleNorm(5)
            );
        }
        */

        if (lineStrength < LINE_DETECT_THRESHOLD) {
            // do not keep building integral when line is lost
            errorIntegral *= 0.98f;

            // line lost: search in last seen direction
            if (lastError < 0.0f) {
                // line was on left -> turn left to search
                setMotorBipolar(PWM_CENTER - SEARCH_OFFSET,
                                PWM_CENTER + SEARCH_OFFSET);
            } else {
                // line was on right -> turn right to search
                setMotorBipolar(PWM_CENTER + SEARCH_OFFSET,
                                PWM_CENTER - SEARCH_OFFSET);
            }
        } else {
            lastError = filteredError;

            // integrate only when error is moderate
            if (fabs(filteredError) < I_ACTIVE_ERROR_LIMIT) {
                errorIntegral += filteredError * DT;
            }

            // slight leak so old bias slowly disappears
            errorIntegral *= I_LEAK;

            // clamp integral
            errorIntegral = clampf(errorIntegral, -I_LIMIT, I_LIMIT);

            // PI steering
            float steer = KP_LINE * filteredError + KI_LINE * errorIntegral;
            steer = clampf(steer, -MAX_STEER, MAX_STEER);

            // both wheels move forward around PWM_CENTER
            float leftCmd  = PWM_CENTER + BASE_OFFSET + LEFT_BIAS + 0.015f + steer;
            float rightCmd = PWM_CENTER + BASE_OFFSET + RIGHT_BIAS - steer;

            setMotorBipolar(leftCmd, rightCmd);
        }

        wait_ms(5);
    }
}
