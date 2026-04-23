#include "mbed.h"

class Bluetooth {
private:
    Serial hm10;
    Ticker tick;
    bool can_send;
    void set_can_send(void){
        can_send=1;
    }
    float clamp(float in, float max, float min){
        if (in > max){return max;}
        if (in < min){return min;}
        return in;
    }

public:
    Bluetooth() : hm10(PA_11, PA_12) {
        tick.attach(callback(this, &Bluetooth::set_can_send), 0.1);
    }

    void begin() {
        hm10.baud(9600);
    }

    void sendWords(const char* text) {
        hm10.printf("%s\r\n", text);
        can_send=0;
    }

    void sendSpeed(float rpmL, float rpmR) {
        hm10.printf("L=%.1f,R=%.1f\r\n", rpmL, rpmR);
        can_send=0;
    }

    void sendTCRT(float LLL, float LL, float L, float R, float RR, float RRR){
        LLL = clamp(LLL, 0.99, 0)*100;
        LL = clamp(LL, 0.99, 0)*100;
        L = clamp(L, 0.99, 0)*100;
        R = clamp(R, 0.99, 0)*100;
        RR = clamp(RR, 0.99, 0)*100;
        RRR = clamp(RRR, 0.99, 0)*100;
        hm10.printf(".%d.%d.%d.%d.%d.%d\r\n", int(LLL), int(LL), int(L), int(R), int(RR), int(RRR));
    }

    bool sendAvailable(void){
        return can_send;
    }

    bool available() {
        return hm10.readable();
    }

    char readChar() {
        return hm10.getc();
    }
};
