#ifndef IIRFILTER_H
#define IIRFILTER_H

#include "CircularBuffer.h"
#include <Arduino.h>

//#define IIRDEBUG

template<size_t order>
class IIRFilter
{
public:
    IIRFilter();
    IIRFilter(float* a_coefficients, float* b_coefficients);
    IIRFilter(double* a_coefficients, double* b_coefficients);

    void setCoefficients(float* a_coefficients, float* b_coefficients);
    void setCoefficients(double* a_coefficients, double* b_coefficients);

    void compute();

    void inline computePeriodic();

    void setInput(float input);
    void setInput(double input);

    float getOutput();

    float input = 0;
    float output = 0;

    unsigned long period = 0; //us

private:

    unsigned long lasttime = 0;

    static constexpr size_t length = order + 1;

    float a_coefficients[order + 1];
    float b_coefficients[order + 1];

    CircularBuffer<float, order + 1> buffer;

};

#include "IIRFilter.tpp"
#endif //IIRFILTER_H