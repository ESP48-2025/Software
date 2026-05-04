#ifndef MOTORCLASS_H
#define MOTORCLASS_H

#include "mbed.h"
#include "EncoderClass.h"

class Motor {
private:
    PwmOut pwm;
    DigitalOut dir;
    DigitalOut mode;

    Encoder encoder;

    float last_command;

public:
    Motor(PinName pwmPin,
          PinName dirPin,
          PinName modePin,
          PinName encA,
          PinName encB,
          float encoderSampleFreq)
        : pwm(pwmPin),
          dir(dirPin),
          mode(modePin),
          encoder(encA, encB, encoderSampleFreq)
    {
        pwm.period_us(50);       // 20 kHz PWM
        mode.write(0);           // 0 = unipolar, same as friend code
        dir.write(1);            // default forward

        last_command = 0.0f;
        setSpeed(0.0f);
    }

    // Friend-style interface
    void begin(void) {
        encoder.begin();
    }

    // Friend-style interface
    void setSpeed(float speed) {
        if (speed < 0.0f) speed = 0.0f;
        if (speed > 1.0f) speed = 1.0f;

        last_command = speed;

        // Important: keep friend's inverted PWM polarity
        pwm.write(1.0f - speed);
    }

    // Keep your old interface as alias
    void speed(float percent_speed) {
        setSpeed(percent_speed);
    }

    // Friend-style interface
    void setDirection(bool direction) {
        dir.write(direction);
    }

    // Keep your old interface as alias
    void is_forward(bool direction) {
        setDirection(direction);
    }

    void bipolar_mode(bool tf) {
        mode.write(tf);
    }

    void pwm_period(float us) {
        pwm.period_us((int)us);
    }

    float get_duty(void) {
        return pwm.read();
    }

    float getCommand(void) {
        return last_command;
    }

    bool get_dir(void) {
        return dir.read();
    }

    bool get_mode(void) {
        return mode.read();
    }

    // Friend-style speed feedback, unit = m/s
    float readSpeedMs(void) {
        return encoder.readSpeedMs();
    }

    // Friend-style pulse reading for turnaround
    int getPulses(void) {
        return encoder.getPulses();
    }

    void resetEncoder(void) {
        encoder.reset();
    }
};

#endif
