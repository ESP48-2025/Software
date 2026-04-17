#include "mbed.h"
#include "Bluetooth.h"
#include "EncoderClass.h"
#include "MotorClass.h"
#include "PID.h"

DigitalOut motorEnable(PA_13, 0);
Motor motorLeft(D15, PC_9, PC_5);
Motor motorRight(D14, PC_8, PC_6);
Bluetooth ble;   
Encoder encR(PC_10, PC_12, 200);
Encoder encL(PC_11, PD_2, 200);

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
    PID LeftMotor(50);
    PID RightMotor(50);
    ble.begin();

    float rpsL, rpsR, pwmL, pwmR;
    
    wait(3);
    //setup time
    motorEnable.write(1);
    const float loop_time_s = 0.02f;
    while (1) {
        LeftMotor.setReference(4);
        LeftMotor.setDT(loop_time_s);
        LeftMotor.setGain(0.025, 0.01, 0);
        rpsL = 0 - encL.getRps();
        pwmL = LeftMotor.updatePID(rpsL) + 0.5;     // offset at 0.5 duty
        
        RightMotor.setReference(4);
        RightMotor.setDT(loop_time_s);
        RightMotor.setGain(0.01875, 0.0125, 0);
        rpsR = encR.getRps();
        pwmR = RightMotor.updatePID(rpsR) + 0.5;     // offset at 0.5 duty
        
        if (ble.sendAvailable()){
            ble.sendSpeed(rpsL, rpsR);
        }

        pwmL = clamp(pwmL, 1, 0);
        motorLeft.speed(pwmL);

        pwmR = clamp(pwmR, 1, 0);
        motorRight.speed(pwmR);


        wait(loop_time_s);
    }
};




