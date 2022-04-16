#include <foc_controller_tmc2160.h>

FOCController::FOCController() {
    init_sine_quart();
}
FOCController::FOCController(MotorEncoder* m_encoder, TMC2160Stepper* driver,
    int16_t max_current_mA, float max_torque, SemaphoreHandle_t SPI_mutex) :
    motor_encoder(m_encoder), driver(driver), max_current_mA(max_current_mA),
    max_torque(max_torque), foc_spi_mutex(SPI_mutex)
{

    this->foc_output_const = (255.0 / 8192.0) * 1.0 / max_torque;

    this->foc_torque_command_spinlock = portMUX_INITIALIZER_UNLOCKED;

    init_sine_quart();
}

void FOCController::setup_driver() {

    Serial.println("Enter FOC Setup");

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    Serial.println("Initialize stepper driver");

    driver->begin();           //  SPI: Init CS pins and possible SW SPI pins
    Serial.println("Talking to driver");

    Serial.print("DRV_STATUS=0b");
    Serial.println(driver->DRV_STATUS(), BIN);

    driver->toff(5);           // Enables driver in software
    driver->rms_current(max_current_mA); // Set motor RMS current
    driver->microsteps(256);   // Set microsteps to 1/16th

    driver->en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
    driver->pwm_autoscale(true); // Needed for stealthChop

    driver->direct_mode(true);

    Serial.println("finished driver setup");
    xSemaphoreGive(foc_spi_mutex);

    Serial.print("DRV_STATUS=0b");
    Serial.println(driver->DRV_STATUS(), BIN);

    init_sine_quart();

    //foc_output_const = (255.0 / 16834.0) * (0.45 / (double(max_current_mA) / 1000.0));
}

void FOCController::calibrate_phase_angle(uint32_t phase_angle_null) {

    if (phase_angle_null != 0) {
        phase_null_angle = phase_angle_null;
    }
    else {
        Serial.println("Starting calibration");
        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        SPI.begin();
        driver->coil_A(0);
        driver->coil_B(0);
        xSemaphoreGive(foc_spi_mutex);

        vTaskDelay(500 / portTICK_PERIOD_MS);

        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        driver->coil_A(255);
        driver->coil_B(0);
        xSemaphoreGive(foc_spi_mutex);

        Serial.println("Position motor");

        vTaskDelay(250 / portTICK_PERIOD_MS);

        uint32_t null_angle_temp = 0.0;
        int count = 100000;

        for (int i = 0; i < count; i++) {
            null_angle_temp = null_angle_temp + motor_encoder->read_angle_raw();
            xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
            xSemaphoreGive(foc_spi_mutex);
        }

        null_angle_temp = null_angle_temp / count;

        phase_null_angle = null_angle_temp;
        //phase_null_angle = FOC_EMPIRIC_PHASE_ANGLE_OFFSET;
    }



#ifdef FOC_DEBUG
    Serial.println("Calibrated Phase Angle");
    Serial.println(phase_null_angle);
    delay(1000);
#endif

}

void FOCController::set_target_torque(float torque_target) {

    if (torque_target > max_torque) {
        torque_target = max_torque;
    }
    else if (torque_target < -max_torque) {
        torque_target = -max_torque;
    }

    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = torque_target;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);

    if (target_torque == 0.0) {
        driver->freewheel(0b00);
    }
}

int32_t sign(int32_t value) {
    if (value >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void FOCController::foc_control() {

    static int32_t angle_diff_prev = 0;
    static const float alpha = 0.5;
    static const int N = N_pole_pairs;

    // obtain current motor angle
    motor_encoder->read_angle_raw();

    // calculate angle_shift
    int32_t angle_diff = int32_t(motor_encoder->_current_angle_raw) - int32_t(motor_encoder->_previous_angle_raw);

    if (motor_encoder->check_rollover()) {
        angle_diff = angle_diff_prev;
    }

    int32_t angle_diff_filtered = angle_diff * (1.0 - alpha) + angle_diff_prev * alpha;
    angle_diff_prev = angle_diff_filtered;

    // anticipate phase shift based on previous shift
    int32_t empiric_phase_shift = angle_diff_filtered * 2.0;

    // calculate desired phase currents
    int16_t i_a_simp = double(-foc_output_const * target_torque * sine_lookup(N * int32_t(int32_t(motor_encoder->_current_angle_raw) - int32_t(phase_null_angle + phase_offset_correct) + empiric_phase_shift)));
    int16_t i_b_simp = double(foc_output_const * target_torque * sine_lookup(4096 + N * int32_t(int32_t(motor_encoder->_current_angle_raw) - int32_t(phase_null_angle + phase_offset_correct) + empiric_phase_shift)));

    // create phase current register values
    xdirect_tmc direct;
    direct.values.coil_A = i_a_simp;
    direct.values.coil_B = i_b_simp;

    // write into current register of the driver
    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->XDIRECT(direct.reg);
    xSemaphoreGive(foc_spi_mutex);

}


void FOCController::set_target_torque_9bit(uint16_t torque_target) {
    static const float inv_max_torque = 1.0 / max_torque;
    if (torque_target > 255) {
        torque_target = 255;
    }
    else if (torque_target < -255) {
        torque_target = -255;
    }
    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = float(torque_target / 255) * inv_max_torque;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
}

void FOCController::init_sine_quart() {

    const double sine_res = FOC_SINE_LOOKUP_RES;

    for (int i = 0; i < FOC_SINE_LOOKUP_SIZE; i++) {
        sineQuart_14bit[i] = sin(sine_res * (PI / 180.0) * double(i)) * 8192;
        //Serial.println(sineQuart[i]);
        //Serial.println(sineQuart_14bit[i]);
        //Serial.println(i);

    }
}


int32_t FOCController::sine_lookup(int32_t val) {
    static const double divisor = FOC_SINE_LOOKUP_DIVISOR_disc;
    int32_t sign = 1;
    if (val < 0) {
        val = -val;
        sign = -1;
    }

    while (val > 16384) {
        val = val - 16384;
    }

    if (val <= 4096) {
        uint16_t index = round(val * divisor);
        return sineQuart_14bit[index] * sign;
    }
    else if (val > 4096 && val <= 8192) {
        uint16_t index = round((val - 4096) * divisor);
        return sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    else if (val > 8192 && val <= 12288) {
        uint16_t index = round((val - 8192) * divisor);
        return (-1) * sineQuart_14bit[index] * sign;
    }
    else if (val > 12288) {
        uint16_t index = round((val - 12288) * divisor);
        return -sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    return 0;

}