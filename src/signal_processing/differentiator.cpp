#include <signal_processing/differentiator.h>
#include <Arduino.h>

Differentiator::Differentiator() {
    /*
    buffer.unshift(0);
    buffer.unshift(0);
    buffer.unshift(0);

    for (int i = 0; i < smoother_length; i++)
    {
        smoother.unshift(0);
    }
    */
};
/*
Differentiator::Differentiator(float timestep)
{
    Differentiator();
    this->timestep = timestep;
    timestep_micro = timestep * 1000 * 1000;
}
*/
Differentiator::Differentiator(double frequency)
{
    // timestep = double(1 / frequency);
    // setTimeStep(timestep);
    setFrequency(frequency);
    Serial.println(timestep);
    Serial.println(timestep_micro);
}

void Differentiator::setTimeStep(double timestep)
{
    timestep_micro = timestep * 1000 * 1000;
    this->timestep = timestep;
}
void Differentiator::setTimeStep(int timestep)
{
    this->timestep_micro = timestep;
    timestep = timestep_micro / (1000.0 * 1000.0);
}
void Differentiator::setFrequency(double frequency)
{
    timestep = double(1.0 / (double)frequency);
    setTimeStep(timestep);
}

void Differentiator::setInput(float input)
{
    // buffer.unshift(input);

    // Filter input
    input = input * alpha + x_tminus * (1 - alpha);

    //input = nonlinearExponentialFilter(input, 0.01, 0.05);

    x_t_array[4] = x_t_array[3];
    x_t_array[3] = x_t_array[2];
    x_t_array[2] = x_t_array[1];
    x_t_array[1] = x_t_array[0];
    x_t_array[0] = input;


    x_tplus = x_t;
    x_t = x_tminus;
    x_tminus = input;
}

void Differentiator::differentiateRobustly() {
    static const double divisor = 1.0 / ((timestep) * 8.0);

    float output_dif1_temp = (2 * (x_t_array[1] - x_t_array[3]) + (x_t_array[0] - x_t_array[4])) * divisor;

    float sigma = 0.01;
    output_dif1_temp = output_dif1_temp * sigma + (1.0 - sigma) * output_dif1;

    if (abs(output_dif1_temp) > 360) {
        return;
    }
    else {
        output_dif1 = output_dif1_temp;
    }


}

void Differentiator::differentriateRobustlyTwice() {
    static const double divisor = 1.0 / (4.0 * timestep * timestep);

    float output_dif = ((x_t_array[0] + x_t_array[4]) - 2.0 * x_t_array[2]) * divisor;

    acc_buffer.unshift(output_dif);
    float intermediate = 0;
    for (int i = 0; i < 10; i++) {
        intermediate += acc_buffer[i];
    }

    output_dif2 = intermediate / 10;


}

void Differentiator::differentiate()
{
    // float y_timeStepPlus = buffer.first();
    // float y_timeStepMinus = buffer.pop();

    static const float divisor = 1.0 / (timestep * 2.0);

    float y_timeStepPlus = x_tplus;
    float y_timeStepMinus = x_tminus;

    float rollover_correction = 0;

    /*
    if ((y_timeStepMinus < 360.0 && y_timeStepMinus >= 180.0) && y_timeStepPlus < 90.0) {
        rollover_correction = 360.0;
    }
    if ((y_timeStepMinus <= 170.0) && y_timeStepPlus > 270.0) {
        rollover_correction = -360.0;

    }
    */




    float output_dif_new = (double)((y_timeStepPlus - y_timeStepMinus + rollover_correction) * divisor);

    if (output_dif_new > 18000.0) {
        output_dif1 = 18000.0;
    }
    else if (output_dif_new < -18000.0) {
        output_dif1 = -18000.0;
    }

    output_dif1 = output_dif_new * alpha + output_dif1 * (1 - alpha);


    /*
    smoother.unshift(output);
    for (int i = 0; i < smoother_length; i++)
    {
        intermediate += smoother[i];
    }
    */
    /*
     alpha = 0.01;
     output = intermediate * alpha + output * (1 - alpha);
     */

     // output = intermediate;

     // output = intermediate / smoother_length;
}

float Differentiator::getFirstDerivativeOutput()
{
    return output_dif1;
}

float Differentiator::getSecondDerivativeOutput() {
    return output_dif2;
}

void Differentiator::differentiateTwice()
{
    static const double divisor = 1.0 / (timestep * timestep);
    float output_dif2_new = (double)(x_tplus - 2 * x_t + x_tminus) * divisor;


    output_dif2 = output_dif2_new * 0.1 + output_dif2 * (1 - 0.1);

    /*

    output = intermediate;
    alpha = 0.01;
    output = intermediate * alpha + output * (1 - alpha);
    */
}

float Differentiator::nonlinearExponentialFilter(float input, const float alpha_low, const float alpha_high, float delta_low_, float delta_high_)
{
    {
        static float output_filt = 0;
        static float delta_low = delta_low_;
        static float delta_high = delta_high_;

        float delta = fabs((input - output_filt) / input);

        //Serial.println(delta);

        if (delta > delta_high)
        {
            alpha = alpha_high;
        }
        else if (delta < delta_low)
        {
            alpha = alpha_low;
        }
        else
        {
            alpha = alpha_low + ((alpha_high - alpha_low) / (delta_high - delta_low)) * delta;
        }

        // Serial.println(delta);
        // Serial.println(alpha);

        output_filt = input * alpha + (1 - alpha) * output_filt;

        return output_filt;
    }
}

void Differentiator::setAlpha(float alpha)
{
    this->alpha = alpha;
}
