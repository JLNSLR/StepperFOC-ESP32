#include <foc_controller_tmc2160.h>

FOCController::FOCController() {
    init_sine_quart();
}
FOCController::FOCController(MotorEncoder* m_encoder, TMC2160Stepper* driver,
    int16_t max_current_mA, float torque_motor_constant, SemaphoreHandle_t SPI_mutex) :
    motor_encoder(m_encoder), driver(driver), max_current_mA(max_current_mA),
    torque_motor_constant(torque_motor_constant), foc_spi_mutex(SPI_mutex)
{

    this->inverse_torque_constant = 1.0 / torque_motor_constant;
    this->current_scale_constant = 255.0 * 1000.0 / max_current_mA;

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
    driver->rms_current(1500); // Set motor RMS current
    driver->microsteps(256);   // Set microsteps to 1/16th

    driver->en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
    // driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
    driver->pwm_autoscale(true); // Needed for stealthChop

    driver->direct_mode(true);

    Serial.println("finished driver setup");
    xSemaphoreGive(foc_spi_mutex);

    Serial.print("DRV_STATUS=0b");
    Serial.println(driver->DRV_STATUS(), BIN);

    init_sine_quart();

    //foc_output_const = (255.0 / 16834.0) * (0.45 / (double(max_current_mA) / 1000.0));
}

void FOCController::calibrate_phase_angle() {

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
    int count = 10000;

    for (int i = 0; i < count; i++) {
        null_angle_temp = null_angle_temp + motor_encoder->read_angle_raw();
        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        xSemaphoreGive(foc_spi_mutex);
    }

    null_angle_temp = null_angle_temp / count;

    phase_null_angle = null_angle_temp;
    phase_null_angle = 15703;

#ifdef FOC_DEBUG
    Serial.println("Calibrated Phase Angle");
    Serial.println(phase_null_angle);
#endif

}

void FOCController::set_target_torque(float torque_target) {
    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = torque_target;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
}

void FOCController::foc_control() {

    motor_encoder->read_angle_raw(); //long time_2 = micros();

    static int32_t angle_diff_prev = 0;
    static float alpha = 0.5;

    int32_t angle_diff = int32_t(motor_encoder->_current_angle_raw) - int32_t(motor_encoder->_previous_angle_raw);

    int32_t angle_diff_filtered = angle_diff * (1.0 - alpha) + angle_diff_prev * alpha;
    const int32_t N = 50;
    angle_diff_prev = angle_diff_filtered;

    int32_t empiric_phase_shift = +angle_diff * 2;
    /*
    if (empiric_phase_shift > 81) {
        empiric_phase_shift = 81;
    }
    else if (empiric_phase_shift < -81) {
        empiric_phase_shift = -81;
    }*/

    /*
   double angle_des = +double(M_ENC_RAW2DEG) * double(int32_t(motor_encoder->_current_angle_raw) - int32_t(phase_null_angle) + empiric_phase_shift);
    */

    float torque = 0.45;
    int16_t i_a_simp = double(-foc_output_const * torque * sine_lookup_(50 * int32_t(int32_t(motor_encoder->_current_angle_raw) - int32_t(phase_null_angle) + empiric_phase_shift)));
    int16_t i_b_simp = double(foc_output_const * torque * sine_lookup_(4096 + 50 * int32_t(int32_t(motor_encoder->_current_angle_raw) - int32_t(phase_null_angle) + empiric_phase_shift)));

    //double i_a = -(0.45 / k_m) * sin(N * angle_des);
    //double i_b = (0.45 / k_m) * cos(N * angle_des);

    //double i_a = +current_const * sin((N * angle_des) * DEG2RAD);
    //double i_b = -current_const * cos((N * angle_des) * DEG2RAD);


    //int16_t i_a_mA = 255 * (i_a / 1.5);
    //int16_t i_b_mA = 255 * (i_b / 1.5);

    //int16_t i_a_9bit = i_a * current_scale_constant;
    //int16_t i_b_9bit = i_b * current_scale_constant;


    xdirect_tmc direct;
    //direct.values.coil_A = i_a_9bit;
    //direct.values.coil_B = i_b_9bit;
    direct.values.coil_A = i_a_simp;
    direct.values.coil_B = i_b_simp;


    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->XDIRECT(direct.reg);
    xSemaphoreGive(foc_spi_mutex);




}


void FOCController::_test_sineLookup(float input) {

    float rad_val = input * (PI / 180);
    long t1 = micros();
    float true_val = sin(rad_val);
    long t2 = micros();
    Serial.println(true_val);
    Serial.println(t2 - t1);

    t1 = micros();
    float est_val = sine_lookup(input);
    t2 = micros();
    Serial.println(est_val);
    Serial.println(t2 - t1);

}

void FOCController::init_sine_quart() {

    const double sine_res = FOC_SINE_LOOKUP_RES;

    for (int i = 0; i < FOC_SINE_LOOKUP_SIZE; i++) {
        sineQuart[i] = sin(sine_res * (PI / 180.0) * i);
        sineQuart_14bit[i] = sin(sine_res * (PI / 180.0) * double(i)) * 8192;
        //Serial.println(sineQuart[i]);
        //Serial.println(sineQuart_14bit[i]);
        //Serial.println(i);

    }
}

double FOCController::sine_lookup(double val) {

    static const double divisor = FOC_SINE_LOOKUP_DIVISOR;

    double sign = 1.0;

    if (val < 0) {
        val = -val;
        sign = -1.0;
    }

    while (val > 360.0) {
        val = val - 360.0;
    }

    if (val <= 90.0) {
        uint16_t index = round(val * divisor);

        return sineQuart[index] * sign;
    }
    else if (val > 90.0 && val <= 180.0) {

        uint16_t index = round((val - 90.0) * divisor);
        return sineQuart[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    else if (val > 180.0 && val <= 270.0) {
        uint16_t index = round((val - 180.0) * divisor);
        return (-1.0) * sineQuart[index] * sign;
    }
    else if (val > 270.0) {
        uint16_t index = round((val - 270.0) * divisor);
        return -sineQuart[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    return 0.0;
}

int32_t FOCController::sine_lookup_(int32_t val) {
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