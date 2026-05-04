#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID {
private:
    // Gains
    float kp;
    float ki;
    float kd;

    // Anti-windup back-calculation gain
    float kaw;

    // Derivative filter time constant
    float t;

    // Sample time
    float Ts;

    // Output limits
    float out_min;
    float out_max;

    // Optional reference for old-style updatePID()
    float reference;

    // Internal states
    float integrator;
    float error_old;
    float differentiator;
    float measurement_old;

    float clamp(float value, float minValue, float maxValue) {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }

public:
    // =====================================================
    // Friend-style constructor
    // =====================================================
    PID(float kp,
        float ki,
        float kd,
        float t,
        float Ts,
        float out_min,
        float out_max,
        float kaw)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->t = t;
        this->Ts = Ts;

        this->out_min = out_min;
        this->out_max = out_max;

        this->kaw = kaw;

        reference = 0.0f;

        integrator = 0.0f;
        error_old = 0.0f;
        differentiator = 0.0f;
        measurement_old = 0.0f;
    }

    // =====================================================
    // Optional old-style constructor
    // This is only for compatibility.
    // If using friend's logic, normally do not use this.
    // =====================================================
    PID(size_t unused_window_size = 1)
    {
        kp = 1.0f;
        ki = 0.0f;
        kd = 0.0f;

        t = 0.05f;
        Ts = 0.02f;

        out_min = -1.0f;
        out_max = 1.0f;

        kaw = 0.6f;

        reference = 0.0f;

        integrator = 0.0f;
        error_old = 0.0f;
        differentiator = 0.0f;
        measurement_old = 0.0f;
    }

    // =====================================================
    // Friend-style update function
    // =====================================================
    float update(float setpoint, float measurement)
    {
        if (Ts <= 0.0f) {
            return 0.0f;
        }

        float error = setpoint - measurement;

        // Proportional
        float proportional = kp * error;

        // Integral using trapezoidal rule
        integrator += 0.5f * ki * Ts * (error + error_old);

        // Derivative on measurement with low-pass filter
        differentiator =
            -(2.0f * kd * (measurement - measurement_old)
              + (2.0f * t - Ts) * differentiator)
            / (2.0f * t + Ts);

        float output_unsat = proportional + integrator + differentiator;

        // Saturation
        float output = clamp(output_unsat, out_min, out_max);

        // Back-calculation anti-windup
        integrator += kaw * Ts * (output - output_unsat);

        error_old = error;
        measurement_old = measurement;

        return output;
    }

    // =====================================================
    // Reset
    // =====================================================
    void reset()
    {
        integrator = 0.0f;
        error_old = 0.0f;
        differentiator = 0.0f;
        measurement_old = 0.0f;
    }

    // =====================================================
    // Friend-style setters
    // =====================================================
    void setGains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void setAntiGain(float kaw)
    {
        this->kaw = kaw;
    }

    // =====================================================
    // Extra setters for compatibility
    // =====================================================
    void setGain(float KP, float KI, float KD)
    {
        setGains(KP, KI, KD);
    }

    void setDT(float DT)
    {
        if (DT > 0.0f) {
            Ts = DT;
        }
    }

    void setReference(float ref)
    {
        reference = ref;
    }

    void setOutputLimit(float minValue, float maxValue)
    {
        out_min = minValue;
        out_max = maxValue;
    }

    void setDerivativeFilter(float T)
    {
        if (T > 0.0f) {
            t = T;
        }
    }

    // =====================================================
    // Old-style updatePID compatibility
    // measurement = feedback
    // pwm parameter is ignored
    // =====================================================
    float updatePID(float measurement, float pwm)
    {
        return update(reference, measurement);
    }
};

#endif
