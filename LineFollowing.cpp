#include "mbed.h"
#include "Bluetooth.h"
#include "EncoderClass.h"
#include "MotorClass.h"
#include "PID.h"

DigitalOut motorEnable(PA_13, 0);
Motor motorLeft(D15, PC_9, PC_5);
Motor motorRight(D14, PC_8, PC_6);
Bluetooth ble;   
Encoder encL(PC_10, PC_12, 200);
Encoder encR(PC_11, PD_2, 200);

float clamp(float value, float range_high, float range_low){
    if (value < range_low){
        return range_low;
    }
    else{
        if (value > range_high){
        return range_high;
        }
        else {
        return value;
        }
    }
}

int main(){
    // float RPS_Max = 4.0;
    // straight line
    PID LeftMotor(3, 5);
    
    float rpmL, rmpR, pwmL, pwmR;
    wait(3);
    //setup time
    motorEnable.write(1);
    while (1) {
        rpmL = encL.getRpm();

    }
};




