#include "mbed.h"

class Motor{
    private:
    PwmOut pwm, dir;
    DigitalOut mode;
public:
    Motor(PinName p, PinName d, PinName m):
        pwm(p), dir(d), mode(m) {
            pwm_period(50);
            mode.write(1);
            speed(0.5);
        }

    float get_duty(void)             { return pwm; }
    float get_dir(void)              { return dir; }
    bool  get_mode(void)             { return mode; }

    void  bipolar_mode(bool tf)      { mode.write(tf); }
    void  pwm_period(float us)       { pwm.period_us(us);}
    void  is_forward(bool direction) { dir.write(direction); }
    void  speed(float percent_speed) { pwm.write(percent_speed); }
};
