#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "mbed.h"

class Bluetooth {
private:
    Serial hm10;
    Ticker tick;

    volatile bool can_send;

    void set_can_send(void) {
        can_send = true;
    }

    float clamp(float in, float max, float min) {
        if (in > max) {
            return max;
        }
        if (in < min) {
            return min;
        }
        return in;
    }

public:
    Bluetooth() : hm10(PA_11, PA_12) {
        can_send = true;
    }

    void begin() {
        hm10.baud(9600);

        // allow sending once every 0.1 s
        tick.attach(callback(this, &Bluetooth::set_can_send), 0.1f);
    }

    void sendWords(const char* text) {
        if (!can_send) return;

        hm10.printf("%s\r\n", text);
        can_send = false;
    }

    void sendSpeed(float speedL, float speedR) {
        if (!can_send) return;

        hm10.printf("L=%.3f,R=%.3f\r\n", speedL, speedR);
        can_send = false;
    }

    void sendTCRT(float LLL, float LL, float L, float R, float RR, float RRR) {
        if (!can_send) return;

        LLL = clamp(LLL, 0.99f, 0.0f) * 100.0f;
        LL  = clamp(LL,  0.99f, 0.0f) * 100.0f;
        L   = clamp(L,   0.99f, 0.0f) * 100.0f;
        R   = clamp(R,   0.99f, 0.0f) * 100.0f;
        RR  = clamp(RR,  0.99f, 0.0f) * 100.0f;
        RRR = clamp(RRR, 0.99f, 0.0f) * 100.0f;

        hm10.printf(".%d.%d.%d.%d.%d.%d\r\n",
                    int(LLL), int(LL), int(L),
                    int(R), int(RR), int(RRR));

        can_send = false;
    }

    bool sendAvailable(void) {
        return can_send;
    }

    bool available(void) {
        return hm10.readable();
    }

    char readChar(void) {
        return hm10.getc();
    }
};

#endif
