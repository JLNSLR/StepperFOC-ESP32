#ifndef FOC_CONTROLLER_TMC2160_H
#define FOC_CONTROLLER_TMC2160_H

#include <motor_encoder.h>
#include <FreeRTOS.h>
#include <TMCStepper.h>
#include <joint_control_global_def.h>

#define FOC_SINE_LOOKUP_RES 1.25    
#define FOC_SINE_LOOKUP_SIZE 72
#define FOC_SINE_LOOKUP_DIVISOR 0.8 // 1/2°

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

    float phase_null_angle = 0;
    float inverse_torque_constant = 1.0 / torque_motor_constant;

    float current_scale_constant = 255 * 1000 / max_current_mA;


    portMUX_TYPE foc_torque_command_spinlock;
    SemaphoreHandle_t foc_spi_mutex;

    void init_sine_quart();
    float sine_lookup(float val);

    float sineQuart[FOC_SINE_LOOKUP_SIZE] = { 0.0 };
};

#endif // ! FOC_CONTROLLER_H
