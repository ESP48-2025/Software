#include "mbed.h"

class Motor{
    private:
        PwmOut pwm, dir;
        DigitalOut mode, enable;
    public:
        Motor(PinName p, PinName d, PinName m, PinName e):
        pwm(p), dir(d), mode(m), enable(e){
            enable = 0;
            
        }
};
//does this upload eygwabrilkgwgeiuaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa

