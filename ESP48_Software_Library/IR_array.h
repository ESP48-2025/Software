#include "mbed.h"

class TCRT{
/*
To use, run update_sample in your main loop.
Then collect values from getCurrentSampleNorm or getCurrentSampleVolts
*/
private:                                                                
    AnalogIn a0, a1, a2, a3, a4, a5;                                    
    float VDD, sample_freq;                                             
    // float currentSampleNorm, currentSampleVolts
    volatile float tcrt_array_norm[6], tcrt_array_volt[6];
    Ticker sampler;
    volatile bool sample_flag;

    void trigger_sample(void){
        sample_flag = 1;
    }

public:                                                                 
    TCRT(PinName pin0, PinName pin1, PinName pin2, PinName pin3, PinName pin4, PinName pin5, float f):
                a0(pin0),       a1(pin1),     a2(pin2),     a3(pin3),       a4(pin4),   a5(pin5), sample_freq(f) {
        sample_flag = 0;
        sampler.attach(callback(this, &TCRT::trigger_sample), 1.0f/sample_freq);
        VDD = 3.3;
    }   
                                                                        
    void set_vdd(float vdd){
        VDD = vdd;
    }

    void sample(void){                                                  
        tcrt_array_norm[0] = a0.read();                                 //Read a sample from the ADC and store normalised representation [0..1]
        tcrt_array_norm[1] = a1.read();
        tcrt_array_norm[2] = a2.read();
        tcrt_array_norm[3] = a3.read();
        tcrt_array_norm[4] = a4.read();
        tcrt_array_norm[5] = a5.read();

        for (int i = 0; i < 6; i++){
            tcrt_array_volt[i] = tcrt_array_norm[i] * VDD;              //Convert this to a voltage and store that as a data member too.
        }               
    }

    const float getCurrentSampleNorm(int tcrt_index){                   //Public member function to return the most recent normalised sample [0..1]
        return tcrt_array_norm[tcrt_index];                             //Return the most recent normalised sample
    }

    const float getCurrentSampleVolts(int tcrt_index){                  //Public member function to return the most recent sampled voltage [0.. 3.3 V]
        return tcrt_array_volt[tcrt_index];                             //Return the most recent sampled voltage
    }

    void update_sample(void){
        if (sample_flag == 1){
            sample_flag = 0;
            sample();
        }
    }
};


class Darlington{
    private:
        DigitalOut en0, en1, en2, en3, en4, en5;
    public:
        Darlington(PinName LLL, PinName LL, PinName L, PinName R, PinName RR, PinName RRR): en0(LLL), en1(LL), en2(L), en3(R), en4(RR), en5(RRR)
        {
            writeAll(0, 0, 0, 0, 0, 0);
        }
        void writeAll(bool Left_most, bool pin1, bool pin2, bool pin3, bool pin4, bool Right_most){
            en0.write(Left_most);
            en1.write(pin1);
            en2.write(pin2);
            en3.write(pin3);
            en4.write(pin4);
            en5.write(Right_most);
        }
        void writeSingle(int pin_from_left, bool logic_value){
            switch (pin_from_left){
                case 0:
                    en0.write(logic_value);
                    break;
                case 1:
                    en1.write(logic_value);
                    break;
                case 2:
                    en2.write(logic_value);
                    break;
                case 3:
                    en3.write(logic_value);
                    break;
                case 4:
                    en4.write(logic_value);
                    break;
                case 5:
                    en5.write(logic_value);
                    break;
                default:
                    break;
            }
        }
};



