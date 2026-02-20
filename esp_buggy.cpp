#include "mbed.h"
#include "C12832.h"

C12832 lcd(D11, D13, D12, D7, D10);


class Motor{
    private:
        PwmOut pwm, dir;
        DigitalOut mode, enable;
    public:
        Motor(PinName p, PinName d, PinName m, PinName e):
        pwm(p), dir(d), mode(m), enable(e){
            enable = 0;
        }
        float get_duty(void){return pwm;};
        float get_dir(void){return dir;};
        bool get_mode(void){return mode;};
        bool get_enable(void){return enable;};

        void enable_motors(bool tf){
            enable.write(tf);
        }
        void bipolar_mode(bool tf){
            mode.write(tf);
        }
        void is_forward(bool direction){
            dir.write(direction);
        }
        void speed(float percent_speed){
            pwm.write(percent_speed);
        }
};

            //pwm, dir, mode, enable
// Motor motorLeft(D11, D10, D8, D7); 
// Motor motorRight(D9, D6, D4, D7);


            //pwm, dir, mode, enable
Motor motorLeft(PB_13, PB_14, PC_5, PA_15);  //D3-D7 pwm, 
Motor motorRight(PB_15, PB_1, PC_6, PA_15);   //PH1, PH0, PC15, PC14


void turn_off_motors(){
    motorLeft.enable_motors(0);
    motorRight.enable_motors(0);
}

Timeout count_down;


int main(void){
    // count_down.attach(&turn_off_motors, 10.0f);
    motorLeft.bipolar_mode(1);
    motorRight.bipolar_mode(1);

    motorLeft.enable_motors(1);
    motorRight.enable_motors(1);
    
    while(1){
        motorLeft.speed(1);
        motorRight.speed(1);

            motorLeft.bipolar_mode(1);
    motorRight.bipolar_mode(1);

    motorLeft.enable_motors(1);
    motorRight.enable_motors(1);
    }
}
