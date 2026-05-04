#ifndef IR_ARRAY_H
#define IR_ARRAY_H

#include "mbed.h"
#include <math.h>

#define NUM_TCRT 6

class TCRT {
private:
    AnalogIn a0, a1, a2, a3, a4, a5;

    float VDD;
    float sample_freq;

    volatile float tcrt_array_norm[NUM_TCRT];
    volatile float tcrt_array_volt[NUM_TCRT];

    float minVal[NUM_TCRT];
    float maxVal[NUM_TCRT];

    float calibrated[NUM_TCRT];

    float position;
    float lastPosition;

    float weights[NUM_TCRT];

    Ticker sampler;
    volatile bool sample_flag;
    bool sampler_started;

    void trigger_sample(void) {
        sample_flag = true;
    }

    float clampf(float value, float minValue, float maxValue) {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }

    float readRawByIndex(int index) {
        switch (index) {
            case 0: return a0.read();
            case 1: return a1.read();
            case 2: return a2.read();
            case 3: return a3.read();
            case 4: return a4.read();
            case 5: return a5.read();
            default: return 0.0f;
        }
    }

public:
    TCRT(PinName pin0, PinName pin1, PinName pin2,
         PinName pin3, PinName pin4, PinName pin5, float f)
        : a0(pin0), a1(pin1), a2(pin2),
          a3(pin3), a4(pin4), a5(pin5), sample_freq(f)
    {
        VDD = 3.3f;
        sample_flag = false;
        sampler_started = false;

        // Same left-to-right sign convention as friend's LineSensor
        weights[0] = -3.0f;
        weights[1] = -2.0f;
        weights[2] = -1.0f;
        weights[3] =  1.0f;
        weights[4] =  2.0f;
        weights[5] =  3.0f;

        position = 0.0f;
        lastPosition = 0.0f;

        for (int i = 0; i < NUM_TCRT; i++) {
            tcrt_array_norm[i] = 0.0f;
            tcrt_array_volt[i] = 0.0f;
            calibrated[i] = 0.0f;

            minVal[i] = 0.0f;
            maxVal[i] = 1.0f;
        }
    }

    // Friend-style interface
    void start(void) {
        if (!sampler_started) {
            sampler.attach(callback(this, &TCRT::trigger_sample),
                           1.0f / sample_freq);
            sampler_started = true;
        }
    }

    void stop(void) {
        sampler.detach();
        sampler_started = false;
    }

    void set_vdd(float vdd) {
        VDD = vdd;
    }

    void sample(void) {
        tcrt_array_norm[0] = a0.read();
        tcrt_array_norm[1] = a1.read();
        tcrt_array_norm[2] = a2.read();
        tcrt_array_norm[3] = a3.read();
        tcrt_array_norm[4] = a4.read();
        tcrt_array_norm[5] = a5.read();

        for (int i = 0; i < NUM_TCRT; i++) {
            tcrt_array_volt[i] = tcrt_array_norm[i] * VDD;
        }
    }

    void update_sample(void) {
        if (sample_flag) {
            sample_flag = false;
            sample();
            updatePosition();
        }
    }

    void calibrate(void) {
    for (int i = 0; i < NUM_TCRT; i++) {
        minVal[i] = 0.0f;
        maxVal[i] = 1.0f;
    }

    for (int t = 0; t < 3000; t++) {
        for (int i = 0; i < NUM_TCRT; i++) {
            float val = readRawByIndex(i);

            if (val < minVal[i]) minVal[i] = val;
            if (val > maxVal[i]) maxVal[i] = val;
        }

        wait_us(1000);
    }
}

    float getCalibrated(int index) {
        if (index < 0 || index >= NUM_TCRT) {
            return 0.0f;
        }

        float raw = tcrt_array_norm[index];

        float val = (raw - minVal[index]) /
                    (maxVal[index] - minVal[index]);

        val = clampf(val, 0.0f, 1.0f);

        calibrated[index] = val;

        return val;
    }

    void updatePosition(void) {
        float sum = 0.0f;
        float total = 0.0f;

        for (int i = 0; i < NUM_TCRT; i++) {
            float val = getCalibrated(i);

            sum += val * weights[i];
            total += val;
        }

        if (total > 0.001f) {
            position = sum / total;
            lastPosition = position;
        } else {
            position = lastPosition;
        }
    }

    float getPosition(void) {
        update_sample();
        return position;
    }

    bool isLineLost(void) {
        update_sample();

        float total = 0.0f;

        for (int i = 0; i < NUM_TCRT; i++) {
            total += getCalibrated(i);
        }

        return (total < 0.8f);
    }

    const float getCurrentSampleNorm(int tcrt_index) {
        if (tcrt_index < 0 || tcrt_index >= NUM_TCRT) {
            return 0.0f;
        }

        return tcrt_array_norm[tcrt_index];
    }

    const float getCurrentSampleVolts(int tcrt_index) {
        if (tcrt_index < 0 || tcrt_index >= NUM_TCRT) {
            return 0.0f;
        }

        return tcrt_array_volt[tcrt_index];
    }

    const float getCurrentCalibratedNorm(int tcrt_index) {
        if (tcrt_index < 0 || tcrt_index >= NUM_TCRT) {
            return 0.0f;
        }

        return calibrated[tcrt_index];
    }
};

// =====================================================
// Darlington class
// =====================================================

class Darlington {
private:
    DigitalOut en0, en1, en2, en3, en4, en5;

public:
    Darlington(PinName LLL, PinName LL, PinName L,
               PinName R, PinName RR, PinName RRR)
        : en0(LLL), en1(LL), en2(L),
          en3(R), en4(RR), en5(RRR)
    {
        writeAll(0, 0, 0, 0, 0, 0);
    }

    void writeAll(bool Left_most, bool pin1, bool pin2,
                  bool pin3, bool pin4, bool Right_most)
    {
        en0.write(Left_most);
        en1.write(pin1);
        en2.write(pin2);
        en3.write(pin3);
        en4.write(pin4);
        en5.write(Right_most);
    }

    void writeSingle(int pin_from_left, bool logic_value) {
        switch (pin_from_left) {
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

#endif
