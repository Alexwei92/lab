#pragma once

#include <ros/ros.h>
#include <cmath>

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        float dt,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_dt(dt)
        , m_integral(0)
        , m_previousError(0)
        , m_previous_LP_Diff(0)
    {
    }

    void updateparam(float kp, float kd, float ki, float minOutput, float maxOutput, float integratorMin, float integratorMax, float dt)
    {
        m_kp = kp;
        m_kd = kd;
        m_ki = ki;
        m_minOutput = minOutput;
        m_maxOutput = maxOutput;
        m_integratorMin = integratorMin;
        m_integratorMax = integratorMax;
        m_dt = dt;
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previous_LP_Diff = 0;
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    float update(float value, float targetValue)
    {
        float ratio = 0.7;
        // error = target - true
        float error = targetValue - value;
        m_integral += error * m_dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        float p = m_kp * error;
        float d = 0;
        float m_difference = (error - m_previousError) / m_dt;
        if (m_dt > 0)
        {
            // A low-pass filter
            m_previous_LP_Diff = ratio*m_difference + (1.0-ratio)*m_previous_LP_Diff;
            d = m_kd * m_previous_LP_Diff;
        }
        float i = m_ki * m_integral;
        float output = p + d + i;
        m_previousError = error;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

    const float get_ki()
    {
        return m_ki;
    }

    const float get_integral()
    {
        return m_integral;
    }

    const float get_Diff()
    {
        return m_previous_LP_Diff;
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_dt;
    float m_integral;
    float m_previousError;
    float m_previous_LP_Diff;

};
