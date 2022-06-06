#include <motor_encoder.h>
#include <math.h>

MotorEncoder::MotorEncoder(AS5048A* encoder, float n_transmission, SemaphoreHandle_t SPI_mutex) :
    magnetic_encoder(encoder), n_transmission(n_transmission), spi_mutex(SPI_mutex)
{
    this->magnetic_encoder = encoder;
    this->n_transmission = n_transmission;
    this->transmission_divider = 1.0 / n_transmission;
    this->pos_input_filter_m.setCoefficients(m_encoder_pos_filter_coefficients_a, m_encoder_pos_filter_coefficients_b);
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
    magnetic_encoder->printErrors();
    magnetic_encoder->printState();
    Serial.println(magnetic_encoder->getErrors());
    this->_current_angle_raw = magnetic_encoder->getRotationPos();
    xSemaphoreGive(spi_mutex);
    Serial.println("Succesfully initialized Motor Encoder...");
}
float MotorEncoder::get_motor_angle_deg() {
    return float(this->_current_angle_raw) * M_ENC_RAW2DEG;
}


float MotorEncoder::get_angle_deg()
{
    float angle_deg = current_shaft_angle_deg;

    float angle_shifted = angle_deg - zero_angle_shaft_deg;

    if (angle_shifted < 180.0) {
        angle_deg = angle_shifted;
    }
    else {
        angle_deg = angle_shifted - 360.0;
    }

    return angle_deg;

}

double MotorEncoder::get_motor_angle_rad() {
    return double(this->_current_angle_raw) * M_ENC_RAW2RAD;
}

float MotorEncoder::get_angle_filtered_deg() {
    float shaft_angle_deg = get_angle_deg();

    // SKip filter when rollover occurs
    pos_input_filter_m.input = shaft_angle_deg;
    pos_input_filter_m.compute();

    /*
    if (shaft_angle_deg < 360.0 - zero_angle_shaft_deg && shaft_angle_deg > 360 - zero_angle_shaft_deg - 1.0) {
        pos_input_filter_m.output = shaft_angle_deg;
    }
    else if (shaft_angle_deg < -zero_angle_shaft_deg + 1) {
        pos_input_filter_m.output = shaft_angle_deg;
    }

    if (pos_input_filter_m.output > 360.0 - zero_angle_shaft_deg) {
        pos_input_filter_m.output = 360.0 - zero_angle_shaft_deg;
    }
    else if (pos_input_filter_m.output < 0.0 - zero_angle_shaft_deg) {
        pos_input_filter_m.output = 0.0 - zero_angle_shaft_deg;
    }
    */

    return pos_input_filter_m.output;
}

void MotorEncoder::set_zero_angle_deg(float zero_angle_deg) {
    this->zero_angle_shaft_deg = zero_angle_deg;
}

float MotorEncoder::get_angle_rad()
{
    portENTER_CRITICAL(&pos_input_data_spinLock);
    float angle_rad = (current_shaft_angle_deg - zero_angle_shaft_deg) * DEG2RAD;
    portEXIT_CRITICAL(&pos_input_data_spinLock);
    return angle_rad;
}

uint32_t MotorEncoder::read_angle_raw(bool averaging)
{

    _previous_angle_raw = _current_angle_raw;
    uint32_t angle_raw;
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    if (averaging) {

        uint32_t angle_raw_array[3];
        angle_raw_array[0] = magnetic_encoder->getRotationPos();
        angle_raw_array[1] = magnetic_encoder->getRotationPos();
        angle_raw_array[2] = magnetic_encoder->getRotationPos();


        //angle_raw = (angle_raw_array[0] + angle_raw_array[1] + angle_raw_array[2] + angle_raw_array[3]) / 4;

        for (int i = 0; i < 3; i++) {
            uint32_t val = angle_raw_array[i];
            if (val > angle_raw_array[i + 1]) {
                angle_raw_array[i] = angle_raw_array[i + 1];
                angle_raw_array[i + 1] = val;
            }
        }

        angle_raw = angle_raw_array[1];

    }
    else {
        angle_raw = magnetic_encoder->getRotationPos();
    }
    xSemaphoreGive(spi_mutex);
    _current_angle_raw = angle_raw;

    if (n_transmission == 1.0)
    { // check wether rollover-detection is necessary
        portENTER_CRITICAL(&pos_input_data_spinLock);
        current_shaft_angle_deg = float(angle_raw) * M_ENC_RAW2DEG - shift_shaft_angle;
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

        if (_rollover) {
            rollover_flag = true;
        }

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
    current_shaft_angle_deg = float(angle_raw) * M_ENC_RAW2DEG * transmission_divider - shift_shaft_angle;
    portEXIT_CRITICAL(&pos_input_data_spinLock);

    return angle_raw;
}

bool MotorEncoder::check_rollover() {

    if (rollover_flag) {
        rollover_flag = false;
        return true;
    }
    else {
        return false;
    }
}
