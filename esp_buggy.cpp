#include "mbed.h"
#include "C12832.h"

class TCRT                                                              //Begin updated potentiometer class definition
{

private:                                                                //Private data member declaration
    AnalogIn a0, a1, a2, a3, a4, a5;                                    //Declaration of AnalogIn object
    float VDD, sample_freq;                                             //Float variable to speficy the value of VDD (3.3 V for the Nucleo-64)
    // float currentSampleNorm, currentSampleVolts
    float tcrt_array_norm[6], tcrt_array_volt[6];
    Ticker sampler;

public:                                                                 // Public declarations
    TCRT(PinName pin0, PinName pin1, PinName pin2, PinName pin3, PinName pin4, PinName pin5, float f):
                a0(pin0),       a1(pin0),     a2(pin0),     a3(pin0),       a4(pin0),   a5(pin0), sample_freq(f) {
        sample_freq = 20000;
        sampler.attach(callback(this, &TCRT::sample), 1/sample_freq);
        VDD = 3.3;
    }   //Constructor - user provided pin name assigned to AnalogIn...
                                                                        //VDD is also provided to determine maximum measurable voltage
    void set_vdd(float vdd){
        VDD = vdd;
    }

    void sample(void){                                                  //Public member function to read a sample and store the value as data members
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



int main(void){
    TCRT IRSensors(A0, A1, A2, A3, A4, A5, 20000);      // Initiate tcrt output and operating frequency (from left most)
    float IRdata[6];
    
    Darlington EnTcrt(PC_8, PA_11, PA_12, D5, D3, D2);  // Initiate enable input (from left most), default 0
    wait(3);

    while(1){
        EnTcrt.writeAll(1, 1, 1, 1, 1, 1);              // enable all

        for (int i = 0; i < 6; i++){                    // read norminal IR sample into an array
            IRdata[i] = IRSensors.getCurrentSampleNorm(i);
        }
    }
    
}
