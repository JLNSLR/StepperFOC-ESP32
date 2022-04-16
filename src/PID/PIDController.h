#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"

#define PID_MODE_INACTIVE 0
#define PID_MODE_ACTIVE 1

#define PID_DIR_DIRECT 0
#define PID_DIR_REVERSE 1

/*
PID Controller class using the fix point math in signed 32-bit arithmetics
*/
class PIDController
{
public:
    PIDController();
    PIDController(float kp, float ki, float kd);

    void compute();
    void computePeriodic();
    void setTuning(float kp, float ki, float kd);

    // set sampleTime in us
    void setSampleTime(int newSampleTime);

    void setOutputLimits(float min, float max);

    void setMode(int Mode);

    void Initialize();

    void SetControllerDirection(int Direction);

    float* getGains();
    // sets the filter for D-Term
    void setDifferentialFilter(bool isActive, float alpha = 0.5);
    // sets the filter for output
    void setOutputFilter(bool isActive, float alpha = 0.5);

    bool getMode();

    void setErrorDeadBand(float const deadband);

    /* ---  working variables --- */

    float input, output, setpoint;
    float outMin, outMax;

private:
    float kp, ki, kd;
    float ki_samp, kd_samp;
    float iTerm, lastInput;

    // sample time in microseconds
    int sampleTime; // microseconds

    unsigned long lastTime;

    bool isActive = true;
    bool filterDerivative = false;

    bool outputFilter = false;

    float errorNoise = 0;

    float errorDeadBand = 0;

    int controllerDirection = PID_DIR_DIRECT;

    float expon_filter_alpha = 0.5;
    float output_exp_filter_alpha = 0.5;
};

#endif // PIDCONTROLLER_H