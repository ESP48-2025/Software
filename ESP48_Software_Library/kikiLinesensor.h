#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "mbed.h"

#define NUM_SENSORS 6

class LineSensor {
private:
    AnalogIn* sensors[NUM_SENSORS];

    float minVal[NUM_SENSORS];
    float maxVal[NUM_SENSORS];

    int weights[NUM_SENSORS];

    float position;        // latest computed position
    float lastPosition;    // for line loss recovery

    Ticker sampler;

public:

    LineSensor(PinName s0, PinName s1, PinName s2,
               PinName s3, PinName s4, PinName s5)
    {
        sensors[0] = new AnalogIn(s0);
        sensors[1] = new AnalogIn(s1);
        sensors[2] = new AnalogIn(s2);
        sensors[3] = new AnalogIn(s3);
        sensors[4] = new AnalogIn(s4);
        sensors[5] = new AnalogIn(s5);

        weights[0] = -3;
        weights[1] = -2;
        weights[2] = -1;
        weights[3] = 1;
        weights[4] = 2;
        weights[5] = 3;

        position = 0.0f;
        lastPosition = 0.0f;
    }

    // ===== START SAMPLING =====
    void start() {
        sampler.attach(callback(this, &LineSensor::update), 0.005f); // 5ms
    }

    // ===== CALIBRATION =====
    void calibrate() {

        for (int i = 0; i < NUM_SENSORS; i++) {
            minVal[i] = 0.0f;
            maxVal[i] = 1.0f;
        }

        for (int t = 0; t < 3000; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                float val = sensors[i]->read();

                if (val < minVal[i]) minVal[i] = val;
                if (val > maxVal[i]) maxVal[i] = val;
            }
            wait_us(1000);
        }
    }

    // ===== BACKGROUND UPDATE =====
    void update() {

        float sum = 0.0f;
        float total = 0.0f;

        for (int i = 0; i < NUM_SENSORS; i++) {
            float val = sensors[i]->read();

            // normalize
            val = (val - minVal[i]) / (maxVal[i] - minVal[i]);

            if (val < 0.0f) val = 0.0f;
            if (val > 1.0f) val = 1.0f;

            sum += val * weights[i];
            total += val;
        }

        if (total > 0.001f) {
            position = sum / total;
            lastPosition = position;
        } else {
            // line lost 鈫?keep last known direction
            position = lastPosition;
        }
    }

    // ===== GET POSITION =====
    float getPosition() {
        return position;
    }

    // ===== LINE LOST DETECTION =====
    bool isLineLost() {
        float total = 0.0f;

        for (int i = 0; i < NUM_SENSORS; i++) {
            float val = sensors[i]->read();
            total += val;
        }

        return (total < 0.5f); // tune threshold
    }
};

#endif
#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "mbed.h"

#define NUM_SENSORS 6

class LineSensor {
private:
    AnalogIn* sensors[NUM_SENSORS];

    float minVal[NUM_SENSORS];
    float maxVal[NUM_SENSORS];

    int weights[NUM_SENSORS];

    float position;        // latest computed position
    float lastPosition;    // for line loss recovery

    Ticker sampler;

public:

    LineSensor(PinName s0, PinName s1, PinName s2,
               PinName s3, PinName s4, PinName s5)
    {
        sensors[0] = new AnalogIn(s0);
        sensors[1] = new AnalogIn(s1);
        sensors[2] = new AnalogIn(s2);
        sensors[3] = new AnalogIn(s3);
        sensors[4] = new AnalogIn(s4);
        sensors[5] = new AnalogIn(s5);

        weights[0] = -3;
        weights[1] = -2;
        weights[2] = -1;
        weights[3] = 1;
        weights[4] = 2;
        weights[5] = 3;

        position = 0.0f;
        lastPosition = 0.0f;
    }

    // ===== START SAMPLING =====
    void start() {
        sampler.attach(callback(this, &LineSensor::update), 0.005f); // 5ms
    }

    // ===== CALIBRATION =====
    void calibrate() {

        for (int i = 0; i < NUM_SENSORS; i++) {
            minVal[i] = 0.0f;
            maxVal[i] = 1.0f;
        }

        for (int t = 0; t < 3000; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                float val = sensors[i]->read();

                if (val < minVal[i]) minVal[i] = val;
                if (val > maxVal[i]) maxVal[i] = val;
            }
            wait_us(1000);
        }
    }

    // ===== BACKGROUND UPDATE =====
    void update() {

        float sum = 0.0f;
        float total = 0.0f;

        for (int i = 0; i < NUM_SENSORS; i++) {
            float val = sensors[i]->read();

            // normalize
            val = (val - minVal[i]) / (maxVal[i] - minVal[i]);

            if (val < 0.0f) val = 0.0f;
            if (val > 1.0f) val = 1.0f;

            sum += val * weights[i];
            total += val;
        }

        if (total > 0.001f) {
            position = sum / total;
            lastPosition = position;
        } else {
            // line lost 鈫?keep last known direction
            position = lastPosition;
        }
    }

    // ===== GET POSITION =====
    float getPosition() {
        return position;
    }

    // ===== LINE LOST DETECTION =====
    bool isLineLost() {
        float total = 0.0f;

        for (int i = 0; i < NUM_SENSORS; i++) {
            float val = sensors[i]->read();
            total += val;
        }

        return (total < 0.05f); // tune threshold
    }
};

#endif
