#include "mbed.h"
// created 28/03/2026
// Author: Yang Cheng
// Requires testing:
/*
Initiate object with reference "reference" and intergration sample size "Isample"
Users might want to create each PID objects with different integration sample sizes,
so a lot of complicated stuff are required to initialize the arrays correctly.
The array initiation still needs testing to see if the array size match.
the integration sample array is made with just pointers, so "sizeof()" cannot be used to determine the size of the array ):
"reference is the tergetted value for the PID to match to"
Users shall use "updatePID()" to insert the new measured value to compare with the reference.
*/

class PID{
    private: 
        float error, reference, output;
        float KP, KI, KD;
        float Inter;
        float* errors;
        size_t Isample;

        void shift(){
            Inter -= 0.5*(errors[Isample-1]+errors[Isample-2]-errors[0]);
            for (int j = Isample-1; j > 0; j--){
            errors[j] = errors[j-1];
            }
            errors[0] = error;
        }

        float integrator(void){
            Inter += 0.5*errors[0];
            return Inter;
        }

        float differentiator(void){
            if (Isample > 1){
                return (errors[0]-errors[1]);
            }
            else{
                return 0;
            }
        }

    public:
        PID(float r, size_t i):reference(r), Isample(i), Inter(0.0f),
        error(0.0f), output(0.0f), KP(1.0f), KI(1.0f), KD(1.0f){
            errors = new float[i];
            for (size_t j = 0; j < Isample; j++){
                errors[j] = 0.0f;
                }

        }
        // a copy constructor to prevent pointer conflicts
        PID(const PID& other):reference(other.reference), Isample(other.Isample), Inter(other.Inter),
        error(other.error), output(other.output), KP(other.KP), KI(other.KI), KD(other.KD){
            errors = new float[Isample];
            for (size_t j = 0; j < Isample; j++){
                errors[j] = other.errors[j];
            }
        }
        // a copy assignment operator to prevent equating conflicts
        PID& operator=(const PID& other){
            if(this != &other){
                delete[] errors;
                reference = other.reference;
                Isample = other.Isample;
                error = other.error;
                Inter = other.Inter;
                output = other.output;
                KP = other.KP;
                KI = other.KI;
                KD = other.KD;
                errors = new float[Isample];
                for (size_t j = 0; j < Isample; j++){
                errors[j] = other.errors[j];
                }
            }
            return *this;
        }
        // destructor to prevent arrays causing corruption
        ~PID(){
            delete[] errors;
        }
        // accessor
        float& operator[](size_t index){
            return errors[index];
        }

        // Actual functions start
        void setGain(float kp, float ki, float kd){
            KP = kp;
            KI = ki;
            KD = kd;
        }
        float updatePID(float newOutput){
            error = newOutput-reference;
            shift();
            output = KP*error + KI*integrator() + KD*differentiator();
            return output;
        }

};




