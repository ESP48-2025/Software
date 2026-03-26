#include "mbed.h"

class Bluetooth {
private:
    Serial hm10;

public:
    Bluetooth() : hm10(PA_11, PA_12) {}

    void begin() {
        hm10.baud(9600);
    }

    void sendWords(const char* text) {
        hm10.printf("%s\r\n", text);
    }

    void sendSpeed(float rpmL, float rpmR) {
        hm10.printf("L=%.1f,R=%.1f\r\n", rpmL, rpmR);
    }

    bool available() {
        return hm10.readable();
    }

    char readChar() {
        return hm10.getc();
    }
};
