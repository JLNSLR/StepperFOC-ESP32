#include <motor_encoder.h>
#include <math.h>

MotorEncoder::MotorEncoder(AS5048A* encoder, uint32_t n_transmission, SemaphoreHandle_t SPI_mutex) :
    magnetic_encoder(encoder), n_transmission(n_transmission), spi_mutex(SPI_mutex)
{
    this->magnetic_encoder = encoder;
    this->n_transmission = n_transmission;
    this->pos_input_filter_m.setCoefficients(m_encoder_pos_filter_coefficients_a, m_encoder_pos_filter_coefficients_b);
    //this->pos_input_filter_m.setCoefficients(m_encoder_pos_filter_coefficients_a, m_encoder_pos_filter_coefficients_b);

    this->pos_input_data_spinLock = portMUX_INITIALIZER_UNLOCKED;

}

void MotorEncoder::init_encoder()
{
#ifdef MOTOR_ENCODER_DEBUG
    Serial.println("Motor Encoder: Iniitialization");
#endif

    Serial.println("Initializing Motor Encoder...");
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    magnetic_encoder->init();
    this->_current_angle_raw = magnetic_encoder->getRotationPos();
    xSemaphoreGive(spi_mutex);
    Serial.println("Succesfully initialized Motor Encoder...");
}
float MotorEncoder::get_motor_angle_deg() {
    return float(this->_current_angle_raw) * M_ENC_RAW2DEG;
}


float MotorEncoder::get_angle_deg()
{
    float angle_deg = current_shaft_angle_deg - zero_angle_shaft_deg;
    return angle_deg;

}

float MotorEncoder::get_angle_filtered_deg() {
    pos_input_filter_m.input = get_angle_deg();
    pos_input_filter_m.compute();
    return pos_input_filter_m.output;
}

float MotorEncoder::get_angle_rad()
{
    portENTER_CRITICAL(&pos_input_data_spinLock);
    float angle_rad = (current_shaft_angle_deg - zero_angle_shaft_deg) * DEG2RAD;
    portEXIT_CRITICAL(&pos_input_data_spinLock);
    return angle_rad;
}

uint32_t MotorEncoder::read_angle_raw()
{

    _previous_angle_raw = _current_angle_raw;
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    uint32_t angle_raw = magnetic_encoder->getRotationPos();
    xSemaphoreGive(spi_mutex);
    _current_angle_raw = angle_raw;

    if (n_transmission == 1)
    { // check wether rollover-detection is necessary
        portENTER_CRITICAL(&pos_input_data_spinLock);
        current_shaft_angle_deg = float(angle_raw) * M_ENC_RAW2DEG;
        portEXIT_CRITICAL(&pos_input_data_spinLock);
        return angle_raw;
    }
    else
    {
        /* perform rollover detection */
        if ((_previous_angle_raw <= _lower_rollover_detect_limit) && (angle_raw >= _upper_rollover_detect_limit))
        {
            _rollover = -1;
        }
        else if ((_previous_angle_raw >= _upper_rollover_detect_limit) && (angle_raw <= _lower_rollover_detect_limit))
        {
            _rollover = 1;
        }
        else
        {
            _rollover = 0;
        }

        _n_full_rotations = _n_full_rotations + _rollover;

        if (_n_full_rotations >= n_transmission)
        {
            _n_full_rotations = 0;
        }
        else if (_n_full_rotations < 0)
        {
            _n_full_rotations = n_transmission - 1;
        }

        angle_raw = angle_raw + encoder_raw_max * _n_full_rotations;
    }

    portENTER_CRITICAL(&pos_input_data_spinLock);
    current_shaft_angle_deg = float(angle_raw) * M_ENC_RAW2DEG;
    portEXIT_CRITICAL(&pos_input_data_spinLock);

    return angle_raw;
}