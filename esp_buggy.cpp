
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
    float getCps() const {return cps;}
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

    int lastL = 0;
    int lastR = 0;

    float rpmL = 0.0f;
    float rpmR = 0.0f;

    float rpmL_buffer, rpmR_buffer;
    

    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Buggy Ready");
    wait_ms(1000);

    while(1) {
        potLeft.sample();
        potRight.sample();

        last_pot_L = potLeft.getCurrentSampleNorm();
        last_pot_R = potRight.getCurrentSampleNorm();
        

        float leftSpeed  = clamp(potLeft.getCurrentSampleNorm(),  0.0f, 1.0f);
        float rightSpeed = clamp(potRight.getCurrentSampleNorm(), 0.0f, 1.0f);

        motorLeft.speed(leftSpeed);
        motorRight.speed(rightSpeed);

        
        rpmL = encLeft.getRpm(CPR);
        rpmR = encRight.getRpm(CPR);

            lcd.cls(); 
            //motor
            lcd.locate(0, 0);
            lcd.printf("L:%.2f R:%.2f", leftSpeed, rightSpeed);
            lcd.locate(64, 0);
            lcd.printf("Lv:%.1fV Rv:%.1fV", potLeft.getCurrentSampleVolts(), potRight.getCurrentSampleVolts());

            //encoder
            lcd.locate(0,20);
            lcd.printf("L rpm:%.1f", rpmL);
            lcd.locate(60,20);
            lcd.printf("R rpm:%.1f", rpmR);

        wait_ms(100);

        // refresh
        // if (abs(rpmL - rpmL_buffer) < 0.05 || abs(rpmR - rpmR_buffer) < 0.05 || abs(potLeft.getCurrentSampleNorm()-last_pot_L) < 0.00005 || abs(potRight.getCurrentSampleNorm()-last_pot_R) < 0.00005){
            
        // }
        // else{
        //     lcd.cls();
        //     //motor
        //     lcd.locate(0, 0);
        //     lcd.printf("L:%.2f R:%.2f", leftSpeed, rightSpeed);
        //     lcd.locate(0, 16);
        //     lcd.printf("Lv:%.1fV Rv:%.1fV", potLeft.getCurrentSampleVolts(), potRight.getCurrentSampleVolts());

        //     //encoder
        //     lcd.locate(0,20);
        //     lcd.printf("L rpm:%.1f", rpmL);
        //     lcd.locate(60,20);
        //     lcd.printf("R rpm:%.1f", rpmR);

        // }
        
        rpmL_buffer = rpmL;
        rpmR_buffer = rpmR;

        // wait_ms(50);
    }
}


