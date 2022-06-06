#include "PIDController.h"

PIDController::PIDController() : input(0), output(0), kp(0), ki(0), kd(0), iTerm(0), lastInput(0), sampleTime(1), lastTime(0), isActive(true) {

};

PIDController::PIDController(float kp, float ki, float kd) : PIDController()
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
};

void PIDController::compute()
{

    if (!isActive)
    {
        return;
    }

    /* Compute error variables */
    float error = setpoint - input;

    if (fabs(error) < errorDeadBand)
    {
        error = 0.0;
    }
    iTerm += ki * error;

    // Clamp iTerm against windup
    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;

    static float prev_dInput = 0;

    float dInput = input - lastInput;

    if (fabs(dInput) < errorDeadBand)
    {
        dInput = 0;
    }

    /* Compute Derivative Filter */
    if (filterDerivative)
    {
        dInput = dInput * expon_filter_alpha + prev_dInput * (1 - expon_filter_alpha);
        prev_dInput = dInput;
    }

    static float prev_output = 0;

    /* Compute PID Output */

    output = kp * error + iTerm - kd * dInput;

    /* Compute Output Filter */
    if (outputFilter)
    {
        output = output * output_exp_filter_alpha + prev_output * (1 - output_exp_filter_alpha);

        prev_output = output;
    }

    // Clamp output against windup
    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    /* Save Variables for next step */
};

void PIDController::computePeriodic()
{
    if (!isActive)
        return;
    unsigned long now = micros();

    int timeChange = now - lastTime;

    if (timeChange >= sampleTime)
    {

        /* Compute error variables */
        float error = setpoint - input;
        iTerm += ki * error;

        // Clamp iTerm against windup
        if (iTerm > outMax)
            iTerm = outMax;
        else if (iTerm < outMin)
            iTerm = outMin;

        float dInput = input - lastInput;

        /* Compute PID Output */

        output = kp * error + iTerm - kd * dInput;

        // Clamp output against windup
        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;

        /* Save Variables for next step */
        lastInput = input;
        lastTime = now;
    }
}

void PIDController::setTuning(float kp, float ki, float kd)
{

    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float sampleTimeInSec = ((float)sampleTime / (1000 * 1000));

    this->kp = kp;
    this->ki = ki * sampleTimeInSec;
    this->kd = kd / sampleTimeInSec;

    if (controllerDirection == PID_DIR_REVERSE)
    {
        this->kp = (0 - kp);
        this->ki = (0 - ki);
        this->kd = (0 - kd);
    }
}

void PIDController::setSampleTime(int newSampleTime)
{

    if (newSampleTime > 0)
    {

        float ratio = (float)newSampleTime / (float)sampleTime;

        ki = (int32_t)((float)ki * ratio);
        kd = (int32_t)((float)kd / ratio);

        sampleTime = newSampleTime;
    }
}

void PIDController::setOutputLimits(float min, float max)
{

    if (min > max)
        return;
    outMin = min;
    outMax = max;

    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::setMode(int Mode)
{
    bool newMode = (Mode == PID_MODE_ACTIVE);
    if (newMode && !isActive)
    { /*we just went from manual to auto*/
        Initialize();
    }
    isActive = newMode;
}

void PIDController::Initialize()
{
    lastInput = input;
    iTerm = output;
    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::SetControllerDirection(int Direction)
{
    controllerDirection = Direction;
}

float* PIDController::getGains()
{

    float sampleTimeInSec = ((float)sampleTime / (1000 * 1000));
    float gains[3] = { kp, ki / sampleTimeInSec, kd * sampleTimeInSec };

    return gains;
}

void PIDController::setDifferentialFilter(bool isActive, float alpha)
{
    this->filterDerivative = isActive;
    this->expon_filter_alpha = alpha;
}

void PIDController::setOutputFilter(bool isActive, float alpha)
{
    this->outputFilter = isActive;
    this->output_exp_filter_alpha = alpha;
}

void PIDController::setErrorDeadBand(float const deadband) {
    this->errorDeadBand = deadband;
}

bool PIDController::getMode()
{
    return this->isActive;
}
