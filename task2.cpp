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
DigitalOut motorEnable(PA_13);

// ---- Motor objects with updated pin assignments ----
// Motor(PWM pin, Direction pin, Mode pin)
Motor motorLeft (D15, PC_9, PC_5);   // D15=pwm1, PC_9=dir1, PC_5=Bi1
Motor motorRight(D14, PC_8, PC_6);   // D14=pwm2, PC_8=dir2, PC_6=Bi2

// ---- Potentiometer objects ----
Potentiometer potLeft (A0, 3.3f);
Potentiometer potRight(A1, 3.3f);

#define CPR 511  

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB, float sam_f)
        : A(pinA), B(pinB), sampling_freq(sam_f), count(0), cps(0.0f)
    {
        A.rise(callback(this, &Encoder::onA));
        A.fall(callback(this, &Encoder::onA));
        sampler.attach(callback(this, &Encoder::updateCps), 1.0f/sampling_freq);
    }

    void reset() { count = 0; }
    int getCount() const { return count; }
    float getCps() const { return cps; }
    float getRpm(int cpr) { return (cps/cpr*60.0f); }

private:
    InterruptIn A;
    InterruptIn B;
    volatile int count;
    float cps;
    Ticker sampler;
    float sampling_freq;

    void onA() {
        if (A.read() == B.read()) count++;
        else count--;
    }
   
    void updateCps() {
        cps = count * sampling_freq;
        count = 0;
    }
};
// ---- Encoder objects with updated pin assignments ----
// Encoder(PinA, PinB, sampling_frequency)
Encoder encLeft(PC_10, PC_12, 20);   // Left encoder: PC10 (enc1A), PC12 (enc1B)
Encoder encRight(PC_11, PD_2, 20);   // Right encoder: PC11 (enc2A), PD2 (enc2B)

int main(void) {
    // Enable both motors via single shared pin
    motorEnable.write(1);

    motorLeft.bipolar_mode(1);
    motorRight.bipolar_mode(1);

    motorLeft.pwm_period(50);
    motorLeft.is_forward(1);

    motorRight.pwm_period(50);
    motorRight.is_forward(1);

    float last_pot_L, last_pot_R;

    encLeft.reset();
    encRight.reset();

    int lastL = 0;
    int lastR = 0;

    float rpmL = 0.0f;
    float rpmR = 0.0f;

    float rpmL_buffer, rpmR_buffer;
   
    // Velocity calculation parameters
    float wheelRadius = 0.040f;  // meters (40mm wheel radius)
    float velocityL = 0.0f;
    float velocityR = 0.0f;

    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Buggy Ready");
    wait_ms(1000);

    while(1) {
        potLeft.sample();
        potRight.sample();

        last_pot_L = potLeft.getCurrentSampleNorm();
        last_pot_R = potRight.getCurrentSampleNorm();
       

        float leftSpeed  = clamp(potRight.getCurrentSampleNorm(),  0.0f, 1.0f);
        float rightSpeed = clamp(potLeft.getCurrentSampleNorm(), 0.0f, 1.0f);

        motorLeft.speed(leftSpeed);
        motorRight.speed(rightSpeed);

       
        rpmL = encLeft.getRpm(CPR);
        rpmR = encRight.getRpm(CPR);

        // Calculate velocity: v = (rpm / 60) * 2π * r
        velocityL = (rpmL / 60.0f) * 2.0f * 3.14159f * wheelRadius;
        velocityR = (rpmR / 60.0f) * 2.0f * 3.14159f * wheelRadius;
        if (velocityR != 0.0f) velocityR = -velocityR;  // Negate only if moving

        lcd.cls();
       
        // Row 1: Motor duty cycles
        lcd.locate(0, 0);
        lcd.printf("L:%.2f R:%.2f", leftSpeed, rightSpeed);
       
        // Row 2: Potentiometer voltages
        lcd.locate(64, 0);
        lcd.printf("Lv:%.1fV Rv:%.1fV", potLeft.getCurrentSampleVolts(), potRight.getCurrentSampleVolts());

        // Row 3: Velocity in m/s
        lcd.locate(0, 20);
        lcd.printf("L:%.2f m/s", velocityL);
        lcd.locate(60, 20);
        lcd.printf("R:%.2f m/s", velocityR);
       
       

        wait_ms(100);
       
        rpmL_buffer = rpmL;
        rpmR_buffer = rpmR;
   
    }
}

