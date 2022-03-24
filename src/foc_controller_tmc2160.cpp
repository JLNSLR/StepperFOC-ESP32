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
    driver->coil_A(0);
    driver->coil_B(-255);
    xSemaphoreGive(foc_spi_mutex);

    Serial.println("Position motor");

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    float null_angle_temp = 0.0;
    int count = 200;

    for (int i = 0; i < count; i++) {
        motor_encoder->read_angle_raw();
        null_angle_temp = null_angle_temp + motor_encoder->get_motor_angle_deg();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    null_angle_temp = null_angle_temp / float(count);

    phase_null_angle = null_angle_temp;

#ifdef FOC_DEBUG
    Serial.println("Calibrated Phase Angle");
#endif

}

void FOCController::set_target_torque(float torque_target) {
    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = torque_target;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
}

void FOCController::foc_control() {

    motor_encoder->read_angle_raw(); //
    double angle_des = motor_encoder->get_motor_angle_deg();

    float N = 50.0;
    /*
    Serial.println("---");
    Serial.println(motor_encoder->get_motor_angle_deg());

    Serial.println(angle_des);
    */

    portENTER_CRITICAL(&foc_torque_command_spinlock);
    float current_scaler = target_torque * inverse_torque_constant;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
    float phase_angle = angle_des * N;


    /*
    Serial.println(inverse_torque_constant);
    Serial.println(current_scaler);
    Serial.println("-");
    Serial.println(sine_lookup(phase_angle));
    Serial.println(sine_lookup(phase_angle + 90.0));
    */

    float i_a = -current_scaler * sine_lookup(phase_angle);
    float i_b = current_scaler * sine_lookup(90 - phase_angle);

    /*
    Serial.println(i_a);
    Serial.println(i_b);

    Serial.println(current_scale_constant);
    */

    int16_t i_a_9bit_val = i_a * current_scale_constant;
    int16_t i_b_9bit_val = i_b * current_scale_constant;



    float K_m = 0.45;
    N = 50.0;
    const double angleFactor = 3.14159265 / 8192;
    angle_des = -double(angleFactor) * double(motor_encoder->magnetic_encoder->getRawRotation()) + phase_null_angle;

    i_a = -(0.45 / K_m) * sin(N * angle_des);
    i_b = (0.45 / K_m) * cos(N * angle_des);

    int16_t i_a_mA = 255 * i_a / 1.5;
    int16_t i_b_mA = 255 * i_b / 1.5;

    xdirect_tmc direct;
    direct.values.coil_A = i_a_mA;
    direct.values.coil_B = i_b_mA;



    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->coil_A(i_a_mA);
    driver->coil_B(i_b_mA);
    //driver->XDIRECT(direct.reg);
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

    const float sine_res = FOC_SINE_LOOKUP_RES;

    for (int i = 0; i < FOC_SINE_LOOKUP_SIZE; i++) {
        sineQuart[i] = sin(sine_res * (PI / 180.0) * i);
        Serial.println(sineQuart[i]);
    }
}

float FOCController::sine_lookup(float val) {

    static const float divisor = FOC_SINE_LOOKUP_DIVISOR;

    float sign = 1.0;

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