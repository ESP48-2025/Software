// Created: 14/04/2026
// Last Updated: 22/04/2026
// Main Author: Yang Cheng
/*
patch notes:
debugged speed control and steering, uses the 22/04/2026 PID class
*/

#include "mbed.h"
#include "Bluetooth.h"
#include "EncoderClass.h"
#include "MotorClass.h"
#include "PID.h"
#include "IR_array.h"

DigitalOut motorEnable(PA_13, 0);
Motor motorLeft(D15, PC_9, PC_5);
Motor motorRight(D14, PC_8, PC_6);
Bluetooth ble;   
Encoder encR(PC_10, PC_12, 200);
Encoder encL(PC_11, PD_2, 200);

TCRT IRSensors(PC_2, PC_3, A2, A3, A4, A5, 200);
Darlington EnTcrt(D2, D3, D8, D9, PA_15, PA_14);


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
    const float loop_time_s = 0.02f;
    // float RPS_Max = 7.0;
    // speed control
    PID LeftMotor(50);
    LeftMotor.setDT(loop_time_s);
    PID RightMotor(50);
    RightMotor.setDT(loop_time_s);

    float refRpsL, refRpsR;
    float refRps_Max = 5.5f;
    float rpsL, rpsR;
    float pwmL = 0.5;
    float pwmR = 0.5;
    
    // steering control
    float IRdata[6];
    float tcrt_weighted;
    float tcrt_response = 0.0f;
    float tcrt_total;
    PID Steering(50);
    Steering.setReference(3.5);
    Steering.setDT(loop_time_s);
    Steering.setGain(0.02, 0, 0);

    // ble report cycle
    ble.begin();
    unsigned int ble_report_cycle = 0;
    
    wait(3);
    //setup time
    motorEnable.write(1);
    while (1) {
        // speed, straight line
        LeftMotor.setReference(refRpsL);
        LeftMotor.setGain(0.05, 0.3, 0);
        rpsL = 0 - encL.getRps();
        pwmL = LeftMotor.updatePID(refRpsL, pwmL) + 0.513;     // offset at 0.5 duty
        
        RightMotor.setReference(refRpsR);
        RightMotor.setGain(0.045, 0.4, 0);
        rpsR = encR.getRps();
        pwmR = RightMotor.updatePID(refRpsR, pwmR) + 0.5;     // offset at 0.5 duty
        
        // measure wheel speed
        if (ble.sendAvailable()){
            ble.sendSpeed(rpsL, rpsR);
        }

        // safety clamp
        pwmL = clamp(pwmL, 1, 0);
        motorLeft.speed(pwmL);

        pwmR = clamp(pwmR, 1, 0);
        motorRight.speed(pwmR);

        // sample tcrt readings
        IRSensors.update_sample();
        tcrt_weighted = 0;
        tcrt_total = 0;
        for (int i=0;i<6;i++){
            IRdata[i] = IRSensors.getCurrentSampleNorm(i);
            tcrt_weighted += (i+1)*IRdata[i];
            tcrt_total +=IRdata[i];
        }
        if (tcrt_total > 1e-6){
            tcrt_weighted = tcrt_weighted / tcrt_total;
        }
        else {
            tcrt_weighted = 3.5f;
            ble_report_cycle = 16;      // error message code 16: division by 0
        }

        // tcrt PID beta -- questionable
        tcrt_response = Steering.updatePID(tcrt_weighted, tcrt_response/(2*refRps_Max)+0.5);    // second argument scaled to max at 0 and 1; min at 0.5
        tcrt_response = clamp(tcrt_response, refRps_Max, -refRps_Max);
        if (tcrt_response > 0){      // Left drift -> turn right -> reduce right speed
            refRpsL = refRps_Max;
            refRpsR = refRps_Max - tcrt_response;
        }
        else {      // Right drift -> turn left -> reduce left speed;
                    // No drift -> response = 0 -> no effect
            refRpsL = refRps_Max + tcrt_response;
            refRpsR = refRps_Max;
        }

        // bluetooth report
        switch (ble_report_cycle){
            case 0:
                ble.sendWords("Measured RPS");
                ble.sendSpeed(rpsL, rpsR);
                break;
            case 1:
                ble.sendTCRT(IRdata[0], IRdata[1], IRdata[2], IRdata[3], IRdata[4], IRdata[5]);
                break;
            case 2:
                ble.sendWords("Target RPS");
                ble.sendSpeed(refRpsL, refRpsR);
                break;
            case 16:    // error message, do not loop
                ble.sendWords("Steering: tcrt 0 div");
                break;
            default:
                ble.sendWords("nothing to report");
                break;
        }
        ble_report_cycle += 1;
        ble_report_cycle %= 2;  // skips error messages on loops

        wait(loop_time_s);
    }
};
