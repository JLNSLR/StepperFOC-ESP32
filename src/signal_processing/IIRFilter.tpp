#include "signal_processing/IIRFilter.h"
//#define IIRDEBUG

template <size_t order>
IIRFilter<order>::IIRFilter()
{
    for (unsigned int i = 0; i < length; i++)
    {
        buffer.unshift(0);
    }
};

template <size_t order>
IIRFilter<order>::IIRFilter(float* a_coefficients_fl, float* b_coefficients_fl)
{
    if (sizeof(a_coefficients) / sizeof(float) == length && sizeof(b_coefficients) / sizeof(float) == length)
    {
        for (unsigned int i = 0; i < order + 1; i++)
        {
            this->a_coefficients[i] = a_coefficients_fl[i];
            this->b_coefficients[i] = b_coefficients_fl[i];
        }
    }
    for (unsigned int i = 0; i < length; i++)
    {
        buffer.unshift(0);
    }
}
template <size_t order>
IIRFilter<order>::IIRFilter(double* a_coefficients_fl, double* b_coefficients_fl)
{
    if (sizeof(a_coefficients) / sizeof(double) == length && sizeof(b_coefficients) / sizeof(double) == length)
    {
        for (int i = 0; i < order + 1; i++)
        {
            this->a_coefficients[i] = a_coefficients_fl[i];
            this->b_coefficients[i] = b_coefficients_fl[i];
        }
    }
    for (unsigned int i = 0; i < length; i++)
    {
        buffer.unshift(0);
    }
}

template <size_t order>
void IIRFilter<order>::setCoefficients(float* a_coefficients_fl, float* b_coefficients_fl)

{
    if (sizeof(a_coefficients) / sizeof(float) == length && sizeof(b_coefficients) / sizeof(float) == length)
    {
        for (int i = 0; i < order + 1; i++)
        {
            this->a_coefficients[i] = a_coefficients_fl[i];
            this->b_coefficients[i] = b_coefficients_fl[i];
        }
    }
}

template <size_t order>
void IIRFilter<order>::setCoefficients(double* a_coefficients_fl, double* b_coefficients_fl)
{
    if (sizeof(a_coefficients) / sizeof(double) == length && sizeof(b_coefficients) / sizeof(double) == length)
    {
        for (int i = 0; i < order + 1; i++)
        {
            this->a_coefficients[i] = a_coefficients_fl[i];
            this->b_coefficients[i] = b_coefficients_fl[i];
        }
    }
}

template <size_t order>
void IIRFilter<order>::compute()
{

#ifdef IIRDEBUG
    for (int i = 0; i < order + 1; i++)
    {
        Serial.print("a: ");
        Serial.println(a_coefficients[i]);
        Serial.print("b: ");
        Serial.println(b_coefficients[i]);
    }
#endif
    float w_n = 0;
    for (unsigned int n = 1; n < length; n++)
    {
        w_n -= a_coefficients[n] * buffer[n - 1];
    }
    w_n += input;
    w_n = w_n * a_coefficients[0];

    buffer.unshift(w_n);

#ifdef IIRDEBUG
    Serial.print("w_n: ");
    Serial.println(w_n);
#endif
    output = 0;
    for (unsigned int n = 0; n < length; n++)
    {
        output += b_coefficients[n] * buffer[n];
    }
}

template <size_t order>
void IIRFilter<order>::setInput(float input)
{
    this->input = input;
}
template <size_t order>
void IIRFilter<order>::setInput(double input)
{
    this->input = input;
}
template <size_t order>
float IIRFilter<order>::getOutput()
{
    return output;
}
template <size_t order>
void IIRFilter<order>::computePeriodic()
{
    if (micros - lasttime >= this->period)
    {
        lasttime = micros();
        this->compute();
    }
}
