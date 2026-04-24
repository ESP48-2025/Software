#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID
{
public:
    PID(float kp,
        float ki,
        float kd,
        float t,
        float Ts,
        float out_min,
        float out_max,
        float kaw);

    float update(float setpoint, float measurement);
    void reset();
    void setGains(float kp, float ki, float kd);
    void setAntiGain(float kaw);

private:
    // Gains
    float kp;
    float ki;
    float kd;

    // back calculation
    float kaw;

    // Derivative filter
    float t;

    // Sample time
    float Ts;

    // Output limits
    float out_min;
    float out_max;

    // Internal states
    float integrator;
    float error_old;
    float differentiator;
    float measurement_old;
};

// -------- IMPLEMENTATION --------

inline PID::PID(float kp,
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
    this->kaw = kaw;

    this->t = t;
    this->Ts  = Ts;

    this->out_min = out_min;
    this->out_max = out_max;

    integrator = 0.0f;
    error_old = 0.0f;
    differentiator = 0.0f;
    measurement_old = 0.0f;
}

inline float PID::update(float setpoint, float measurement)
{
    if (Ts <= 0.0f)
        return 0.0f;

    float error = setpoint - measurement;

    // Proportional
    float proportional = kp * error;

    // Integral
    integrator += 0.5f * ki * Ts * (error + error_old);

    // Derivative (filtered, on measurement)
    differentiator = -(2.0f * kd * (measurement - measurement_old) +
                      (2.0f * t - Ts) * differentiator) /
                      (2.0f * t + Ts);

    float output = proportional + integrator + differentiator;
    float output_unsat = output;

    // Saturation
    if (output > out_max)
        output = out_max;
    else if (output < out_min)
        output = out_min;

    // Anti-windup
    integrator += kaw * Ts * (output - output_unsat);

    error_old = error;
    measurement_old = measurement;

    return output;
}

inline void PID::reset()
{
    integrator = 0.0f;
    error_old = 0.0f;
    differentiator = 0.0f;
    measurement_old = 0.0f;
}

inline void PID::setGains(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

inline void PID::setAntiGain(float kaw)
{
    this->kaw = kaw;
}

#endif
