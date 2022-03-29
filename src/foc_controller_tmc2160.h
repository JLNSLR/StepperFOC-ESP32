#ifndef FOC_CONTROLLER_TMC2160_H
#define FOC_CONTROLLER_TMC2160_H

#include <motor_encoder.h>
#include <FreeRTOS.h>
#include <TMCStepper.h>
#include <joint_control_global_def.h>

#define FOC_SINE_LOOKUP_RES  0.0219726
#define FOC_SINE_LOOKUP_SIZE 4096
#define FOC_SINE_LOOKUP_DIVISOR 2.84444 // 1/2°
#define FOC_SINE_LOOKUP_DIVISOR_disc 1 // 1/2°

#define EMPIRIC_PHASE_ANGLE_OFFSET 15703 //

#define FOC_CONSTANT 0.069173 // 255/8192*1/0.45 --- 0.45Nm = holding torque

#define FOC_DEBUG


class FOCController
{
public:
    FOCController();
    FOCController(MotorEncoder* m_encoder, TMC2160Stepper* driver, int16_t max_current_mA,
        float torque_motor_constant, SemaphoreHandle_t SPI_mutex);

    void setup_driver();
    void calibrate_phase_angle(); // blocking function should only be called

    void foc_control();

    MotorEncoder* motor_encoder;
    TMC2160Stepper* driver;
    int16_t max_current_mA;
    float torque_motor_constant = 0.45;

    float target_torque = 0.3;

    bool calibrated = false;

    void set_target_torque(float torque_target);
    void _test_sineLookup(float input);

    long microseconds = 0;

private:
    union xdirect_tmc
    {
        uint32_t reg : 25;
        struct
        {
            int16_t coil_A : 9;
            int8_t : 7;
            int16_t coil_B : 9;
        } values;
    }direct;

    uint32_t phase_null_angle = 0;
    float inverse_torque_constant = 1.0 / torque_motor_constant;

    float current_scale_constant = 255 * 1000 / max_current_mA;


    portMUX_TYPE foc_torque_command_spinlock;
    SemaphoreHandle_t foc_spi_mutex;

    void init_sine_quart();
    double sine_lookup(double val);
    int32_t sine_lookup_(int32_t val);

    int32_t predictive_angle_shift = 0;

    double sineQuart[FOC_SINE_LOOKUP_SIZE] = { 0.0 };
    int32_t sineQuart_14bit[FOC_SINE_LOOKUP_SIZE] = { 0 };

    const double foc_output_const = FOC_CONSTANT;
};

#endif // ! FOC_CONTROLLER_H
