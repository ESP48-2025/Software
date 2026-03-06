#include "mbed.h"
#include "C12832.h"
class TCRT                                                   //Begin updated potentiometer class definition
{

private:                                                                //Private data member declaration
    AnalogIn inputSignal;                                               //Declaration of AnalogIn object
    float VDD, currentSampleNorm, currentSampleVolts, sample_freq;                   //Float variable to speficy the value of VDD (3.3 V for the Nucleo-64)
    Ticker sampler;

public:                                                                 // Public declarations
    TCRT(PinName pin, float v, float f) : inputSignal(pin), VDD(v), sample_freq(f) {
        sample_freq = 20000;
        sampler.attach(callback(this, &TCRT::sample), 1/sample_freq);
    }   //Constructor - user provided pin name assigned to AnalogIn...
                                                                        //VDD is also provided to determine maximum measurable voltage
    float amplitudeVolts(void)                                          //Public member function to measure the amplitude in volts
    {
        return (inputSignal.read()*VDD);                                //Scales the 0.0-1.0 value by VDD to read the input in volts
    }

    float amplitudeNorm(void)                                           //Public member function to measure the normalised amplitude
    {
        return inputSignal.read();                                      //Returns the ADC value normalised to range 0.0 - 1.0
    }

    void sample(void)                                                   //Public member function to read a sample and store the value as data members
    {
        currentSampleNorm = inputSignal.read();                         //Read a sample from the ADC and store normalised representation [0..1]
        currentSampleVolts = currentSampleNorm*VDD;                     //Convert this to a voltage and store that as a data member too.
    }

    const float getCurrentSampleNorm(void)                              //Public member function to return the most recent normalised sample [0..1]
    {
        return currentSampleNorm;                                       //Return the most recent normalised sample
    }

    const float getCurrentSampleVolts(void)                             //Public member function to return the most recent sampled voltage [0.. 3.3 V]
    {
        return currentSampleVolts;                                      //Return the most recent sampled voltage
    }

};

class Darlington{
    private:
        BusOut enable_tcrt;
        // DigitalOut en0, en1, en2, en3, en4, en5;

    public:
        Darlington(PinName LLL, PinName LL, PinName L, PinName R, PinName RR, PinName RRR): enable_tcrt(LLL, LL, L, R, RR, RRR)
        // en0(LLL), en1(LL), en2(L), en3(R), en4(RR), en5(RRR)
        
        {
            writeAll(0b000000);
            // en0 = enable_tcrt[0];
            // en1 = enable_tcrt[1];
            // en2 = enable_tcrt[2];
            // en3 = enable_tcrt[3];
            // en4 = enable_tcrt[4];
            // en5 = enable_tcrt[5];
        }
        void writeAll(int zero_b_int){
            enable_tcrt = zero_b_int;
        }
        void writeSingle (int index, int value){
            enable_tcrt[index] = value;
        }
};


int main(void){
    TCRT SenseLLL(A0, 3.3, 20000);
    TCRT SenseLL(A1, 3.3, 20000);
    TCRT SenseL(A2, 3.3, 20000);
    TCRT SenseR(A3, 3.3, 20000);
    TCRT SenseRR(A4, 3.3, 20000);
    TCRT SenseRRR(A5, 3.3, 20000);

    float tcrt_array[6] = {0};

    Darlington EnTcrt(PB_12, PA_11, PA_12, D5, D6, D0);    // 6 pins to sync enable

    wait(3);


    while(1){
        tcrt_array[0] = SenseLLL.getCurrentSampleNorm();
        tcrt_array[1] = SenseLL.getCurrentSampleNorm();
        tcrt_array[2] = SenseL.getCurrentSampleNorm();
        tcrt_array[3] = SenseR.getCurrentSampleNorm();
        tcrt_array[4] = SenseRR.getCurrentSampleNorm();
        tcrt_array[5] = SenseRRR.getCurrentSampleNorm();
    }
    

}
