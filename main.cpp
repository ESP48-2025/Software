#include "mbed.h"
#include "C12832.h"


// ---- LCD ----
C12832 lcd(D11, D13, D12, D7, D10);

// ---- Potentiometer Class ----
class Potentiometer {
private:
    AnalogIn inputSignal;
    float VDD, currentSampleNorm, currentSampleVolts;
public:
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {}

    float amplitudeVolts(void) { return (inputSignal.read() * VDD); }
    float amplitudeNorm(void)  { return inputSignal.read(); }

    void sample(void) {
        currentSampleNorm  = inputSignal.read();
        currentSampleVolts = currentSampleNorm * VDD;
    }
    const float getCurrentSampleNorm(void)  { return currentSampleNorm; }
    const float getCurrentSampleVolts(void) { return currentSampleVolts; }
};

// ---- Motor Class (enable removed — handled globally) ----
class Motor {
private:
    PwmOut pwm, dir;
    DigitalOut mode;
public:
    Motor(PinName p, PinName d, PinName m):
        pwm(p), dir(d), mode(m) {}

    float get_duty(void)             { return pwm; }
    float get_dir(void)              { return dir; }
    bool  get_mode(void)             { return mode; }
    void  bipolar_mode(bool tf)      { mode.write(tf); }
    void  pwm_period(float us)       { pwm.period_us(50);}
    void  is_forward(bool direction) { dir.write(direction); }
    void  speed(float percent_speed) { pwm.write(percent_speed); }
};

// ---- clamp helper ----
float clamp(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

// ---- Single shared enable pin ----
DigitalOut motorEnable(PA_15);

// ---- Motor objects (no enable pin passed) ----
Motor motorLeft (PB_13, PB_14, PC_5);
Motor motorRight(D2, D3, PC_6);

// ---- Potentiometer objects ----
Potentiometer potLeft (A0, 3.3f);
Potentiometer potRight(A1, 3.3f);

#define CPR 512

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB, float sam_f)
        : A(pinA), B(pinB), sampling_freq(sam_f)
    {
        count = 0;
        A.rise(callback(this, &Encoder::onA));
        A.fall(callback(this, &Encoder::onA));
        sampler.attach(callback(this, &Encoder::updateCps), 1/sampling_freq);
    }

    void reset() { count = 0; }
    int getCount() const { return count; }
    float getCps() const {return cps;}
    float getRpm(int cpr) {return (cps/cpr*60);}
    

private:
    InterruptIn A;
    InterruptIn B;
    volatile int count, last_count;
    Ticker sampler;
    float sampling_freq, cps;

    void onA() {
        if (A.read() == B.read()) count++;
        else count--;
    }
    void updateCps() {
        int now = count;
        int delta = now - last_count;
        last_count = now;

        cps = ((float)delta)*sampling_freq;
    }
};

Encoder encLeft(PB_5, PB_7, 20);
Encoder encRight(PB_8, PB_9, 20);

class Joystick {
private:
    DigitalIn fire;
public:
    Joystick(PinName f) : fire(f){};
    bool PressFIRE(void){return fire.read();}
};

#define STRAIGHT_COUNT 1020
#define TURN90_COUNT 280
#define TURN180_COUNT 575

void stop(int ms = 300) {
    motorLeft.speed(0.5f);
    motorRight.speed(0.5f);
    wait_ms(ms);
}

void goStraight() {
    encLeft.reset();
    encRight.reset();

    while ((abs(encLeft.getCount()) + abs(encRight.getCount()))/2 < STRAIGHT_COUNT ){
        const float KP = 0.002f;
        int error = abs(encLeft.getCount() - abs(encRight.getCount()));
        float calibration = error * KP;
        float dutyCycleL = 0.65 - calibration;
        float dutyCycleR = 0.65 + calibration;

        motorLeft.speed(dutyCycleL);
        motorRight.speed(dutyCycleR);
    }

    stop(300);
}

void turnLeft90() {
    encLeft.reset();
    encRight.reset();

    motorLeft.speed(.64f);
    motorRight.speed(.33f);

    while (1) {
        if (abs(encLeft.getCount())  >= TURN90_COUNT) motorLeft.speed(0.5f);
        if (abs(encRight.getCount()) >= TURN90_COUNT) motorRight.speed(0.5f);

        if (abs(encLeft.getCount()) >= TURN90_COUNT && abs(encRight.getCount()) >= TURN90_COUNT) break;
    }

    stop(300);
}

void turnRight90() {
    encLeft.reset();
    encRight.reset();

    motorLeft.speed(.33f);
    motorRight.speed(.7f);

    while (1) {
        if (abs(encLeft.getCount())  >= TURN90_COUNT) motorLeft.speed(0.5f);
        if (abs(encRight.getCount()) >= TURN90_COUNT) motorRight.speed(0.5f);

        if (abs(encLeft.getCount()) >= TURN90_COUNT && abs(encRight.getCount()) >= TURN90_COUNT) break;
    }

    stop(300);
}

// ===== U-turn 180 (use your existing method; same structure as turns) =====
void uTurn() {
    encLeft.reset();
    encRight.reset();

    motorLeft.speed(.33f);
    motorRight.speed(.68f);

    while (1) {
        if (abs(encLeft.getCount())  >= TURN180_COUNT) motorLeft.speed(0.5f);
        if (abs(encRight.getCount()) >= TURN180_COUNT) motorRight.speed(0.5f);

        if (abs(encLeft.getCount()) >= TURN180_COUNT && abs(encRight.getCount()) >= TURN180_COUNT) break;
    }

    stop(300);
}

int main(void) {
    Joystick stick(A2); 
    

    motorEnable.write(0);

    motorLeft.bipolar_mode(1);
    motorRight.bipolar_mode(1);

    motorLeft.pwm_period(50);
    motorRight.pwm_period(50);

    lcd.cls();
    lcd.printf("Square Start");
    int cond = 1;
    wait(2);

    while(cond){
        if (stick.PressFIRE()){
            cond = 0;
        }
    }
    lcd.cls();
    lcd.printf("starting up");

    wait_ms(5000);

    motorEnable.write(1);

    int condition = 1;

    for(int i=0; i<3; i++){
    goStraight();
    turnLeft90();
    }

    goStraight();

    uTurn();

    for(int i=0; i<3; i++){
        goStraight();
        turnRight90();
    }

    goStraight();

    while(1);
}
