#include "mbed.h"

class Encoder {
    private:
        InterruptIn A;
        InterruptIn B;
        volatile int count;
        Ticker sampler;
        float sampling_freq, cps;
        float wheel_size;
        int cpr;

        void onA() {
            if (A.read() == B.read()) count++;
            else count--;
        }
        void getCps() {
            cps = count*sampling_freq;
            reset();
        }

    public:
        Encoder(PinName pinA, PinName pinB, float sam_f)
            : A(pinA), B(pinB), sampling_freq(sam_f)
        {
            count = 0;
            A.rise(callback(this, &Encoder::onA));
            A.fall(callback(this, &Encoder::onA));
            sampler.attach(callback(this, &Encoder::getCps), 1/sampling_freq);
            wheel_size = 0.04;
            cpr = 511;
        }

        void reset() { count = 0; }
        void setwheelsize(float size) { wheel_size = size;}
        void setCpr(int CPR) {cpr = CPR;}

        int getCount() const { return count; }
        float returnCps() const {return cps;}
        float getRpm(void) {return (cps/cpr*60);}
        float getVel(void) const {return cps/cpr*2*3.14159*wheel_size;}
};

