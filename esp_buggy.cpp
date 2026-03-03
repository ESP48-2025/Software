#include "mbed.h"
#include "C12832.h"
#include "mbed2/299/TARGET_NUCLEO_F401RE/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F401RE/PinNames.h"

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


int main(void){
    TCRT SenseLLL(A0, 3.3, 20000);
    TCRT SenseLL(A1, 3.3, 20000);
    TCRT SenseL(A2, 3.3, 20000);
    TCRT SenseR(A3, 3.3, 20000);
    TCRT SenseRR(A4, 3.3, 20000);
    TCRT SenseRRR(A5, 3.3, 20000);
    

}
