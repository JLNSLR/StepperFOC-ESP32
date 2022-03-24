#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H
#include <CircularBuffer.h>
#include <math.h>

class Differentiator
{
public:
    Differentiator();
    // Differentiator(float timestep);
    Differentiator(double frequency);

    void differentiate();
    void differentiateTwice();
    void setTimeStep(double timestep); // seconds
    void setTimeStep(int timestep);    // microseconds
    void setFrequency(double frequency);

    void setInput(float input);
    float getOutput();

    float nonlinearExponentialFilter(float input, const float alpha_low, const float alpha_high, float delta_low_ = 0.1, float delta_high_ = 0.8);

    void setAlpha(float alpha);

private:
    double timestep = 0.003333;

    int timestep_micro = 3333;

    /*
    CircularBuffer<float, 3> buffer;
    int smoother_length = 30;
    CircularBuffer<float, 30> smoother;
    */

    float alpha = 0.5;

    float input = 0;
    float output_dif1 = 0;
    float output_dif2 = 0;

    float x_tminus = 0;
    float x_t = 0;
    float x_tplus = 0;
};

#endif // DIFFERENTIATOR_H