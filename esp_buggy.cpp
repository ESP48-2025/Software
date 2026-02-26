// First revision, no testing yet. Completely beta.
// Created on top of TD1 encoder code
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

#define CPR 511  

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB, float sam_f)
        : A(pinA), B(pinB), sampling_freq(sam_f)
    {
        count = 0;
        A.rise(callback(this, &Encoder::onA));
        A.fall(callback(this, &Encoder::onA));
        sampler.attach(callback(this, &Encoder::getCps), 1/sampling_freq);
    }

    void reset() { count = 0; }
    int getCount() const { return count; }
    float retrieveCps() const {return cps;}
    float getRpm(int cpr) {return (cps/cpr*60);}
    

private:
    InterruptIn A;
    InterruptIn B;
    volatile int count;
    Ticker sampler;
    float sampling_freq, cps;

    void onA() {
        if (A.read() == B.read()) count++;
        else count--;
    }
    void getCps() {
        cps = count*sampling_freq;
        reset();
    }
};

Encoder encLeft(PC_14, PC_15, 20);
Encoder encRight(PB_8, PB_9, 20);


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


    float cpsL = 0.0f;
    float cpsR = 0.0f;

    float pwmL, pwmR;
    float changed_pwm, target_pwm;
    float dutyCorrection = 0.1;
    int correctionScale = 1;
    // float m;        // m = L/R
    float error = 1, error0 = 1, errorLast = 1;    // 1 - m
    bool loop1 = 1;

    float DL_DR_ratio[6];

    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Buggy Ready");
    wait_ms(1000);
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Matching duties:\n\nDL/DR:");

    for (int i = 5; i<11; i++){
        target_pwm = float(i)/10;
        // pwmL = target_pwm;
        // pwmR = target_pwm;
        // lcd.printf("Lduty: %.4f\tRduty: %.4f", pwmL, pwmR);
        lcd.locate(((i-5)/6)*128, 8);
        lcd.printf("%.2f", target_pwm);
        
        motorLeft.speed(target_pwm);
        motorRight.speed(target_pwm);

        // enter calibration loop
        error = 1;
        loop1 = 1;
        while(abs(error) > 0.02){            
            wait(3);

            // sample avearge cps
            cpsL = 0;
            cpsR = 0;
            for (int j = 0; j < 10; j++){
                wait_ms(500);
                cpsL += encLeft.retrieveCps();
                cpsR += encRight.retrieveCps();
            }
            // cpsL /= 10;
            // cpsR /= 10;

            // error decision
            error = 1 - cpsL/cpsR;
            if (loop1 == 1){
                loop1 = 0;
                error0 = error;
                correctionScale = 1;
            }
            else if (errorLast*error < 0){
                correctionScale++;
            }
            changed_pwm -= (error0*error)/abs(error0*error)*dutyCorrection/correctionScale;
            if (error0 < 0){
                pwmL = changed_pwm;
                pwmR = target_pwm;
            }
            else{
                pwmR = changed_pwm;
                pwmL = target_pwm;
            }
            errorLast = error;

            // prep new speed
            motorLeft.speed(pwmL);
            motorRight.speed(pwmR);
            
        };
        // error ~ 0, matched
        DL_DR_ratio[i-5] = pwmL/pwmR;
        lcd.locate(((i-5)/6)*128, 24);
        lcd.printf("%.2f", DL_DR_ratio[i-5]);

    }
    motorEnable.write(0);
}
