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

/* Controllers */

PIDController drvSys_velocity_controller(0.01, 0.0001, 0);
PIDController drvSys_position_controller(10, 10, 0);

/* Differentiators */
Differentiator drvSys_differentiator_motor_pos(DRVSYS_PROCESS_MOTOR_ENCODER_FREQU);


CircularBuffer<float, 10> drvSys_position_buffer;
CircularBuffer<float, 10> drvSys_velocity_buffer;
CircularBuffer<float, 10> drvSys_acc_buffer;
CircularBuffer<float, 10> drvSys_output_torque_buffer;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_motor_encoder_th;
TaskHandle_t drvSys_vel_controller_th;

uint64_t f_count = 0;
uint64_t e_count = 0;

enum drvSys_priorities { foc_prio = 10, process_sensor_prio = 9, acc_control_prio = 8, vel_control_prio = 7, pos_control_prio = 6 };


long elapsedTime_us = 0;

void drvSys_setupDriver() {


#ifdef DRV_SYS_DEBUG
    Serial.println("Setting up Drive System");
#endif // DRV_SYS_DEBUG


    drvSys_motor_encoder.init_encoder();
    drvSys_foc_controller.setup_driver();

    drvSys_foc_controller.calibrate_phase_angle(0);
    Serial.println("finished calibration");

    drvSys_state_flag = ready;

}

void drvSys_initialize() {

    drvSys_setupDriver();

    drvSys_setupInterrupts();

    drvSys_motor_encoder.read_angle_raw();
    float zero_angle = drvSys_motor_encoder.get_angle_deg();
    drvSys_motor_encoder.set_zero_angle_deg(zero_angle);

    /* set up motion controllers */

    drvSys_velocity_controller.setSampleTime(DRVSYS_CONTROL_VEL_PERIOD_US);
    drvSys_velocity_controller.setOutputLimits(DRVSYS_TORQUE_CONSTANT * (-1.0), DRVSYS_TORQUE_CONSTANT);
    drvSys_velocity_controller.setpoint = 0.0;

    drvSys_velocity_controller.Initialize();


    drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);
    drvSys_velocity_controller.SetControllerDirection(PID_DIR_REVERSE);




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
            0 // task handle
        );
        Serial.println("create Task2");
        xTaskCreatePinnedToCore(
            _drvSys_debug_print_position_task,   // function name
            "print_position_debug_task", // task name
            2000,      // Stack size (bytes)
            NULL,      // task parameters
            2,         // task priority
            NULL,
            1 // task handle
        );
        xTaskCreatePinnedToCore(
            _drvSys_read_serial_commands_task,   // function name
            "read serial commands", // task name
            1000,      // Stack size (bytes)
            NULL,      // task parameters
            1,         // task priority
            NULL,
            1 // task handle
        );

        /* Create Controller Tasks */

        /*
        xTaskCreatePinnedToCore(
            _drvSys_velocity_controller_task,   // function name
            "velocity_control_task", // task name
            1000,      // Stack size (bytes)
            NULL,      // task parameters
            vel_control_prio,         // task priority
            &drvSys_vel_controller_th,
            0 // task handle
        );
        */


        xTaskCreatePinnedToCore(
            drvSys_test_signal_task,   // function name
            "print test signal", // task name
            1000,      // Stack size (bytes)
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


        drvSys_foc_controller.set_target_torque(0.0);
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
        //vTaskNotifyGiveFromISR(drvSys_vel_controller_th, &xHigherPriorityTaskWoken);
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
            long time_0 = micros();
            if (xSemaphoreTake(drvSys_mutex_joint_angle_buffer, 10 / portTICK_PERIOD_MS) == pdTRUE) {

                /* Obtain Position Data from Motor Encoder */
                float motor_angle_filtered_deg = drvSys_motor_encoder.get_angle_filtered_deg();
                //motor_angle_filtered_deg = drvSys_motor_encoder.get_angle_deg();
                drvSys_position_buffer.unshift(motor_angle_filtered_deg);
                xSemaphoreGive(drvSys_mutex_joint_angle_buffer);

                /* Perform numerical differentiation on filtered position data */
                drvSys_differentiator_motor_pos.setInput(motor_angle_filtered_deg);
                drvSys_differentiator_motor_pos.differentiateRobustly();
                drvSys_differentiator_motor_pos.differentriateRobustlyTwice();

                /* Save Output of Differentiation to Buffers */
                xSemaphoreTake(drvSys_mutex_joint_vel_buffer, portMAX_DELAY);
                drvSys_velocity_buffer.unshift(drvSys_differentiator_motor_pos.getFirstDerivativeOutput());
                xSemaphoreGive(drvSys_mutex_joint_vel_buffer);

                xSemaphoreTake(drvSys_mutex_joint_acc_buffer, portMAX_DELAY);
                drvSys_acc_buffer.unshift(drvSys_differentiator_motor_pos.getSecondDerivativeOutput());
                xSemaphoreGive(drvSys_mutex_joint_acc_buffer);

            }
            else {
                drvSys_state_flag = error;
                /* Handle Errors of Drive System */
            }
            elapsedTime_us = micros() - time_0;

            /* process position data to obtain derivatives */
            ///TODO
        }

    }
}

void drvSys_setupControllers() {
    // Setup Velocity Controller
    //drvSys_velocity_controller.
}

void _drvSys_debug_print_position_task(void* parameters) {


    Serial.println("angle:,velocity:,acceleration:");
    Serial.println("angle:,velocity:");
    while (true) {

        xSemaphoreTake(drvSys_mutex_joint_angle_buffer, portMAX_DELAY);
        float angle = drvSys_position_buffer.last();
        xSemaphoreGive(drvSys_mutex_joint_angle_buffer);

        xSemaphoreTake(drvSys_mutex_joint_vel_buffer, portMAX_DELAY);
        float vel = drvSys_velocity_buffer.last();
        xSemaphoreGive(drvSys_mutex_joint_vel_buffer);

        xSemaphoreTake(drvSys_mutex_joint_acc_buffer, portMAX_DELAY);
        float acc = drvSys_acc_buffer.last();
        xSemaphoreGive(drvSys_mutex_joint_acc_buffer);


        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print(angle);
        Serial.print(',');
        Serial.print(vel);
        Serial.print(',');
        Serial.println(acc / 100.0);
        //Serial.println(elapsedTime_us);
        xSemaphoreGive(glob_Serial_mutex);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

void _drvSys_read_serial_commands_task(void* parameters) {
    while (true) {

        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        size_t available_bytes = Serial.available();
        if (available_bytes > 0) {
            char* input_buffer;
            String input = Serial.readString();

            xSemaphoreGive(glob_Serial_mutex);

            drvSys_parse_ASCI_command(input);



        }
        xSemaphoreGive(glob_Serial_mutex);

        vTaskDelay(20 / portTICK_PERIOD_MS);

    }
}

void drvSys_parse_ASCI_command(String input) {

    int32_t index_divider = input.indexOf(':');
    String keyword = input.substring(0, index_divider);
    float arguments[10] = { 0 };
    bool end_of_command_reached = 0;
    int32_t n_argument = 0;
    int32_t index = index_divider;
    int32_t next_delimiter_at = input.indexOf(',');
    while (!end_of_command_reached) {

        arguments[n_argument] = input.substring(index + 1, next_delimiter_at).toFloat();
        index = next_delimiter_at;
        next_delimiter_at = input.indexOf(',');
        if (next_delimiter_at == -1) {
            end_of_command_reached = true;
        }
        else {
            n_argument++;
        }
    }

    Serial.print(keyword);
    Serial.print(',');
    Serial.println(arguments[0]);

    if (keyword == "t") {
        float target_torque = arguments[0];
        drvSys_foc_controller.set_target_torque(target_torque);
    }

    if (keyword == "phoff") {
        int32_t phase_offset = arguments[0];
        drvSys_foc_controller.phase_offset_correct = phase_offset;
    }
    if (keyword == "vel_pid") {
        float P = arguments[0];
        float I = arguments[1];
        float D = arguments[2];

        drvSys_velocity_controller.setTuning(P, I, D);
    }

}

void drvSys_test_signal_task(void* parameters) {

    while (true) {
        for (int i = 0; i < 1000; i++) {
            float target_val = 3 * sin(2 * PI * 0.08 * 100 / 1000 * i);

            //drvSys_foc_controller.set_target_torque(target_val);
            drvSys_velocity_controller.setpoint = target_val;

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}


void _drvSys_velocity_controller_task(void* parameters) {

    drvSys_velocity_controller.Initialize();
    drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);

    uint32_t velocity_controller_processing_thread_notification;

    while (true) {

        velocity_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (velocity_controller_processing_thread_notification) {

            xSemaphoreTake(drvSys_mutex_joint_vel_buffer, portMAX_DELAY);
            float actual_vel = drvSys_velocity_buffer.last();
            xSemaphoreGive(drvSys_mutex_joint_vel_buffer);

            drvSys_velocity_controller.input = actual_vel;
            drvSys_velocity_controller.compute();
            float torque_target = drvSys_velocity_controller.output;

            Serial.println(torque_target);

            xSemaphoreTake(drvSys_mutex_joint_torque_buffer, portMAX_DELAY);
            drvSys_foc_controller.set_target_torque(torque_target);
            xSemaphoreGive(drvSys_mutex_joint_torque_buffer);
        }


    }

}



