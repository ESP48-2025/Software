#ifndef ENCODERCLASS_H
#define ENCODERCLASS_H

#include "mbed.h"

class Encoder {
private:
    InterruptIn A;
    InterruptIn B;

    volatile int count;        // counts within one sampling period
    volatile int total_count;  // total accumulated counts

    Ticker sampler;

    float sampling_freq;
    volatile float cps;        // counts per second

    // Match friend's motor-speed conversion
    float wheel_diameter;      // metres
    float gear_ratio;
    int counts_per_motor_rev;

    bool sampler_started;

    void onA() {
        // Direction detection using B phase
        if (A.read() == B.read()) {
            count++;
            total_count++;
        } else {
            count--;
            total_count--;
        }
    }

    void sampleCps() {
        cps = (float)count * sampling_freq;
        count = 0;
    }

public:
    Encoder(PinName pinA, PinName pinB, float sam_f)
        : A(pinA), B(pinB)
    {
        sampling_freq = sam_f;

        count = 0;
        total_count = 0;
        cps = 0.0f;

        // These values are chosen to match your friend's code:
        // speed_ms = rev_per_sec * 0.075 * pi
        // rev_per_sec = tick_rate / (512 * 10.84)
        wheel_diameter = 0.075f;
        gear_ratio = 10.84f;
        counts_per_motor_rev = 512;

        sampler_started = false;

        A.rise(callback(this, &Encoder::onA));
        A.fall(callback(this, &Encoder::onA));
    }

    void begin() {
        if (!sampler_started) {
            sampler.attach(callback(this, &Encoder::sampleCps),
                           1.0f / sampling_freq);
            sampler_started = true;
        }
    }

    void stop() {
        sampler.detach();
        sampler_started = false;
    }

    void reset() {
        count = 0;
        total_count = 0;
        cps = 0.0f;
    }

    void resetTotal() {
        total_count = 0;
    }

    void setWheelDiameter(float diameter_m) {
        wheel_diameter = diameter_m;
    }

    void setGearRatio(float ratio) {
        gear_ratio = ratio;
    }

    void setCountsPerMotorRev(int counts) {
        counts_per_motor_rev = counts;
    }

    int getCount() const {
        return count;
    }

    int getTotalCount() const {
        return total_count;
    }

    // Friend-compatible name
    int getPulses() const {
        return total_count;
    }

    float returnCps() const {
        return cps;
    }

    float getRps() const {
        float counts_per_wheel_rev =
            (float)counts_per_motor_rev * gear_ratio;

        return cps / counts_per_wheel_rev;
    }

    float getRpm() const {
        return getRps() * 60.0f;
    }

    // Friend-compatible speed unit: metres per second
    float readSpeedMs() const {
        float counts_per_wheel_rev =
            (float)counts_per_motor_rev * gear_ratio;

        float rev_per_sec = cps / counts_per_wheel_rev;
        float speed_ms = rev_per_sec * wheel_diameter * 3.1415962f;

        return speed_ms;
    }

    // Keep your old function name as well
    float getVel() const {
        return readSpeedMs();
    }
};

#endif
