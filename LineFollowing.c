#include "mbed.h"
#include "Bluetooth.h"
#include "EncoderClass.h"
#include "MotorClass.h"

DigitalOut motorEnable(PA_13, 0);
Motor motorLeft(D15, PC_9, PC_5);
Motor motorRight(D14, PC_8, PC_6);
Bluetooth ble;   
Encoder encL(PC_10, PC_12, 20000);
Encoder encR(PC_11, PD_2, 20000);

#include <vector>
using namespace std;
class PID{
    private: 
        float error, reference, output;
        float KP, KI, KD;
        int Isample;
        float Inter;
        vector<int> errors;

        float integrator(void){
            Inter = 0;
            for (int j = 0; j < Isample; j++){
                Inter += errors[j];
            }
            return Inter/Isample;
        }

    public:
        PID(float r, int i):reference(r), Isample(i), errors(i){
            error = 0;
            output = 0;
            reference = 0;
            Isample = 5;
            KP = 1;
            KI = 1;
            KD = 1;
        }
        void setGain(float kp, float ki, float kd){
            KP = kp;
            KI = ki;
            KD = kd;
        }
        float updatePID(float newOutput){
            // P
            error = newOutput-reference;
            output = KP*error + //unfinished
        }

};

int main(){
    float RPS_Max = 4.0;
    // straight line

};




