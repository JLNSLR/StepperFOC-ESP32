#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#define MOTOR_ENCODER_DEBUG

// --- CONSTANTS --- //
#define M_ENC_RAW_MAX 16384
#define M_ENC_RAW2DEG 0.02197265625
#define M_ENC_RAW2RAD 3.8349519697141030742952189737e-4

#define M_POS_FILTER_ORDER 4
#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105


#include <signal_processing/IIRFilter.h>
#include <AS5048A.h>
#include <Arduino.h>
#include <FreeRTOS.h>


class MotorEncoder
{
public:
    MotorEncoder();
    MotorEncoder(AS5048A* encoder, float n_transmission, SemaphoreHandle_t SPI_mutex);

    void init_encoder();
    uint32_t read_angle_raw(bool averaging = false); //only function that calls the Sensor!
    float get_angle_deg();
    float get_angle_rad();
    float get_motor_angle_deg();
    double get_motor_angle_rad();
    float get_angle_filtered_deg();
    void set_zero_angle_deg(float zero_angle);

    AS5048A* magnetic_encoder;
    float n_transmission = 1;

    IIRFilter<M_POS_FILTER_ORDER> pos_input_filter_m;
    uint32_t _current_angle_raw = 0;
    uint32_t _previous_angle_raw = 0;

    bool check_rollover();


private:
    float current_shaft_angle_deg = 0.0;
    float zero_angle_shaft_deg = 0.0;

    const float shift_shaft_angle = 180.0;
    float transmission_divider = 1 / n_transmission;

    /* --- */
    int32_t _rollover = 0;
    int32_t _n_full_rotations = 0;

    bool rollover_flag = false;

    const int32_t encoder_raw_max = M_ENC_RAW_MAX;
    int32_t _zero_angle_raw = 0;

    int32_t _upper_rollover_detect_limit = 10000;
    int32_t _lower_rollover_detect_limit = 7000;

    portMUX_TYPE pos_input_data_spinLock;
    SemaphoreHandle_t spi_mutex;

    /* Filter Coefficients */
    float m_encoder_pos_filter_coefficients_a[M_POS_FILTER_ORDER + 1] = { 1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981 };
    float m_encoder_pos_filter_coefficients_b[M_POS_FILTER_ORDER + 1] = { 4.165992044065965e-04,0.001666396817626,0.002499595226440,0.001666396817626,4.165992044065965e-04 };
};

#endif // MOTOR_ENCODER_H