#include <drive_system.h>

#define CS_PIN 17  // Chip select
/*Buffer Spinlocks */
SemaphoreHandle_t drvSys_mutex_joint_angle_buffer = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_vel_buffer = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_acc_buffer = xSemaphoreCreateBinary();;
SemaphoreHandle_t drvSys_mutex_joint_torque_buffer = xSemaphoreCreateBinary();;
SemaphoreHandle_t drvSys_mutex_motor_commanded_torque = xSemaphoreCreateBinary();;
SemaphoreHandle_t drvSys_mutex_position_command = xSemaphoreCreateBinary();;


/*Hardware Timer Setup Variables*/
const uint16_t drvSys_timer_prescaler_divider = DRVSYS_TIMER_PRESCALER_DIV; // with 80MHz Clock, makes the timer tick every 1us
const uint64_t drvSys_timer_alarm_rate_us = DRVSYS_TIMER_ALARM_RATE_US; //generate timer alarm every 50us

hw_timer_t* drvSys_foc_timer;
volatile const  int32_t drvSys_timer_foc_ticks = DRVSYS_FOC_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_encoder_process_ticks = DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_acc_control_ticks = DRVSYS_CONTROL_ACC_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const  int32_t drvSys_timer_vel_control_ticks = DRVSYS_CONTROL_VEL_PERIOD_US / drvSys_timer_alarm_rate_us;

drvSys_StateFlag drvSys_state_flag = not_ready;

/* Drive Components */
AS5048A drvSys_magnetic_motor_encoder(CS_ENCODER);
MotorEncoder drvSys_motor_encoder(&drvSys_magnetic_motor_encoder, DRVSYS_TRANSMISSION_RATIO, glob_SPI_mutex);
TMC2160Stepper drvSys_driver(CS_TMC, R_SENSE);
FOCController drvSys_foc_controller(&drvSys_motor_encoder, &drvSys_driver, DRVSYS_PHASE_CURRENT_MAX_mA, DRVSYS_TORQUE_CONSTANT, glob_SPI_mutex);


CircularBuffer<float, 10> drvSys_position_buffer;
CircularBuffer<float, 10> drvSys_velocity_buffer;
CircularBuffer<float, 10> drvSys_acc_buffer;
CircularBuffer<float, 10> drvSys_output_torque_buffer;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_motor_encoder_th;

uint64_t f_count = 0;
uint64_t e_count = 0;

enum drvSys_priorities { foc_prio = 10, process_sensor_prio = 9, acc_control_prio = 8, vel_control_prio = 7, pos_control_prio = 6 };

void drvSys_setupDriver() {

#ifdef DRV_SYS_DEBUG
    Serial.println("Setting up Drive System");
#endif // DRV_SYS_DEBUG


    drvSys_motor_encoder.init_encoder();
    drvSys_foc_controller.setup_driver();

    drvSys_foc_controller.calibrate_phase_angle();
    Serial.println("finished calibration");

    drvSys_state_flag = ready;

}

void drvSys_initialize() {

    drvSys_setupDriver();

    drvSys_setupInterrupts();

}

void  drvSys_start() {

    Serial.println("create Tasks");
    if (drvSys_state_flag == ready) {
        /* --- create Tasks --- */
        xTaskCreatePinnedToCore(
            _drvSys_foc_controller_task,   // function name
            "FoC-Controller-Task", // task name
            3000,      // Stack size (bytes)
            NULL,      // task parameters
            foc_prio,         // task priority
            &drvSys_foc_th,
            0 // task handle
        );
        Serial.println("create Task1");

        xTaskCreatePinnedToCore(
            _drvSys_process_motor_encoder_task,   // function name
            "Process-motor-encoder-task", // task name
            2000,      // Stack size (bytes)
            NULL,      // task parameters
            2,         // task priority
            &drvSys_process_motor_encoder_th,
            1 // task handle
        );
        Serial.println("create Task2");
        xTaskCreatePinnedToCore(
            _drvSys_debug_print_position_task,   // function name
            "print_position_debug_task", // task name
            2000,      // Stack size (bytes)
            NULL,      // task parameters
            1,         // task priority
            NULL,
            1 // task handle
        );


        xSemaphoreGive(drvSys_mutex_joint_angle_buffer);
        xSemaphoreGive(drvSys_mutex_position_command);
        xSemaphoreGive(drvSys_mutex_joint_acc_buffer);
        xSemaphoreGive(drvSys_mutex_joint_vel_buffer);
        xSemaphoreGive(drvSys_mutex_joint_torque_buffer);
        xSemaphoreGive(drvSys_mutex_motor_commanded_torque);




        // Start Timer
        Serial.println("start interrupt timer");
        timerAlarmEnable(drvSys_foc_timer);


        drvSys_foc_controller.set_target_torque(0.3);
        //flag start
        drvSys_state_flag = active;
    }
}

void drvSys_setupInterrupts() {

    Serial.println("trying to setup interrupts");
    drvSys_foc_timer = timerBegin(0, drvSys_timer_prescaler_divider, true);
    timerAttachInterrupt(drvSys_foc_timer, &drvSys_on_foc_timer, true);
    timerAlarmWrite(drvSys_foc_timer, drvSys_timer_alarm_rate_us, true);
    Serial.println("finished interrupt setup");

}



void IRAM_ATTR drvSys_on_foc_timer() {
    volatile static uint64_t tickCount = 0;

    tickCount++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (tickCount % drvSys_timer_foc_ticks == 0) { // react every 200us ->5kHz
        vTaskNotifyGiveFromISR(drvSys_foc_th, &xHigherPriorityTaskWoken);
    }
    if (tickCount % drvSys_timer_encoder_process_ticks == 0) { //react every 4000us
        vTaskNotifyGiveFromISR(drvSys_process_motor_encoder_th, &xHigherPriorityTaskWoken);
    }
    if (tickCount % drvSys_timer_acc_control_ticks == 0) {
        //vTaskvTaskNotifyGiveFromISR(...)
    }
    if (tickCount % drvSys_timer_vel_control_ticks == 0) {
        //vTaskvTaskNotifyGiveFromISR(...)
    }

    portYIELD_FROM_ISR();
}

void _drvSys_foc_controller_task(void* parameters) {

    uint32_t foc_thread_notification;

    while (true) {

        foc_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (foc_thread_notification) {
            drvSys_foc_controller.foc_control();
        }

    }
}

void _drvSys_process_motor_encoder_task(void* parameters) {

    uint32_t encoder_processing_thread_notification;
    while (true) {
        encoder_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        if (encoder_processing_thread_notification) {
            if (xSemaphoreTake(drvSys_mutex_joint_angle_buffer, 10 / portTICK_PERIOD_MS) == pdTRUE) {

                //drvSys_position_buffer.unshift(drvSys_motor_encoder.get_angle_filtered_deg());
                drvSys_position_buffer.unshift(drvSys_motor_encoder.get_motor_angle_deg());
                xSemaphoreGive(drvSys_mutex_joint_angle_buffer);

            }
            else {
                drvSys_state_flag = error;
                /* Handle Errors of Drive System */
            }

            /* process position data to obtain derivatives */
            ///TODO
        }

    }
}

void _drvSys_debug_print_position_task(void* parameters) {


    while (true) {

        xSemaphoreTake(drvSys_mutex_joint_angle_buffer, portMAX_DELAY);
        float angle = drvSys_position_buffer.last();
        xSemaphoreGive(drvSys_mutex_joint_angle_buffer);


        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.println(angle);
        xSemaphoreGive(glob_Serial_mutex);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}



