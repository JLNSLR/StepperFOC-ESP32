#include <drive_system.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "kinematic_kalman_filter.h"

/* ####################################################################
############### RTOS AND TIMING PARAMETERS ############################
######################################################################*/
/*Buffer Semaphores */
SemaphoreHandle_t drvSys_mutex_motor_position = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_vel = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_acc = xSemaphoreCreateBinary();;

SemaphoreHandle_t drvSys_mutex_joint_position = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_vel = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_acc = xSemaphoreCreateBinary();;

SemaphoreHandle_t drvSys_mutex_joint_torque = xSemaphoreCreateBinary();

SemaphoreHandle_t drvSys_mutex_torque_target = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_commanded_torque = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_position_command = xSemaphoreCreateBinary();;

/*Hardware Timer Setup Variables*/
const uint16_t drvSys_timer_prescaler_divider = DRVSYS_TIMER_PRESCALER_DIV; // with 80MHz Clock, makes the timer tick every 1us
const uint64_t drvSys_timer_alarm_rate_us = DRVSYS_TIMER_ALARM_RATE_US; //generate timer alarm every 50us

hw_timer_t* drvSys_foc_timer;
volatile const  int32_t drvSys_timer_foc_ticks = DRVSYS_FOC_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_encoder_process_ticks = DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_torque_control_ticks = DRVSYS_CONTROL_TORQUE_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const  int32_t drvSys_timer_vel_control_ticks = DRVSYS_CONTROL_VEL_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_pos_control_ticks = DRVSYS_CONTROL_POS_PERIOD_US / drvSys_timer_alarm_rate_us;


/*###########################################################################
################### Drive System Object Definitions #########################
############################################################################*/

/* Drive System General Variables */

/* Drive Constants */

drvSys_Constants drvSys_constants = { .nominal_current_mA = DRVSYS_PHASE_CURRENT_NOMINAL_mA,
    .transmission_ratio = DRVSYS_TRANSMISSION_RATIO,
    .joint_id = JOINT_ID,
    .motor_torque_constant = DRVSYS_TORQUE_CONSTANT };



/* Drive System Control Mode */

drvSys_controlMode drvSys_mode = cascade_position_control;

drvSys_StateFlag drvSys_state_flag = not_ready;


/* Drive State Parameters */
drvSys_controllerState drvSys_controller_state = { .control_mode = drvSys_mode,
.state_flag = drvSys_state_flag,
.calibrated = false,
.overtemperature = false,
.temperature_warning = false,
.temperature = 20 };


struct {
    int target_type; //0 - pos, 1 - vel, 2 - torque
    float max;
    float min;
    int shape; // 0 - rectangle, 1 - sine, 2 - ramp
    float period;
    bool active;
}drvSys_test_signal;


drvSys_parameters drvSys_parameter_config;


/* #####################################################################################
####################### ----- Drive Components  ------ #################################
###################################################################################### */


AS5048A drvSys_magnetic_motor_encoder(CS_ENCODER);
MotorEncoder drvSys_motor_encoder(&drvSys_magnetic_motor_encoder, DRVSYS_TRANSMISSION_RATIO, glob_SPI_mutex);
TMC2160Stepper drvSys_driver(CS_TMC, R_SENSE);
FOCController drvSys_foc_controller(&drvSys_motor_encoder, &drvSys_driver, DRVSYS_PHASE_CURRENT_MAX_mA, DRVSYS_TORQUE_CONSTANT, glob_SPI_mutex);
// Joint Encoder TODO
// Torque Sensor TODO


/* Kinematic Kalman Filter */
KinematicKalmanFilter motor_angle_kalman_filter(DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US * 1e-6);
KinematicKalmanFilter joint_angle_kalman_filter(DRVSYS_PROCESS_JOINT_ENCODER_PERIOD_MS * 1e-3);



/* Differentiators */
Differentiator drvSys_differentiator_motor_pos(DRVSYS_PROCESS_MOTOR_ENCODER_FREQU);

/* ########## Controllers ################ */
PIDController drvSys_velocity_controller(0.00, 0.0000, 0);
PIDController drvSys_position_controller(0, 0, 0);




/* ########### Sensor Buffers ########### */
CircularBuffer<float, 10> drvSys_position_buffer;
CircularBuffer<float, 10> drvSys_velocity_buffer;
CircularBuffer<float, 10> drvSys_acc_buffer;
CircularBuffer<float, 10> drvSys_joint_torque_buffer;



float drvSys_motor_position;
float drvSys_motor_velocity;
float drvSys_motor_acc;

float drvSys_joint_position;
float drvSys_joint_velocity;
float drvSys_joint_acc;

float drvSys_joint_torque;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_motor_encoder_th;
TaskHandle_t drvSys_torque_controller_th;
TaskHandle_t drvSys_vel_controller_th;
TaskHandle_t drvSys_pos_controller_th;
TaskHandle_t drvSys_process_joint_encoder_th;
TaskHandle_t drvSys_process_torque_sensor_th;
TaskHandle_t drvSys_admittance_controller_th;
TaskHandle_t drvSys_test_signal_th;


/* #############################################################
######################## Target values #########################
###############################################################*/
float drvSys_pos_target = 0;
float drvSys_vel_target = 0;
float drvSys_torque_target = 0;
float drvSys_m_torque_commanded = 0;

// used to differentiate between torque_ff and reference torque for admittance control
float drvSys_joint_torque_ref = 0.0;


/*######## Feed Forward targets #############*/
float drvSys_vel_ff = 0.0;
float drvSys_torque_ff = 0.0;


/* ####### Drive System Preferences ########### */

//used to save parameters into flash memory
Preferences drv_sys_preferences;

const char* drvSys_posPID_saved_gains = "posGains";
const char* drvSys_velPID_saved_gains = "velGains";
const char* drvSys_admittance_saved_gains = "admGains";

const char* drvSys_pos_offset = "posOffset";


/* Drive System Priority constants */
enum drvSys_priorities {
    foc_prio = 10, process_sensor_prio = 9, torque_control_prio = 8,
    vel_control_prio = 7, pos_control_prio = 6, admittance_control_prio = 5
};

/* ###############################################################
##################################################################
################## Function Implementations ######################
##################################################################
################################################################ */



drvSys_driveState drvSys_get_drive_state() {
    drvSys_driveState state;
    xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
    state.joint_pos = drvSys_motor_position;
    xSemaphoreGive(drvSys_mutex_motor_position);

    xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
    state.joint_vel = drvSys_motor_velocity;
    xSemaphoreGive(drvSys_mutex_motor_vel);

    xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
    state.joint_acc = drvSys_motor_acc;
    xSemaphoreGive(drvSys_mutex_motor_acc);

    xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
    state.joint_torque = drvSys_joint_torque;
    xSemaphoreGive(drvSys_mutex_joint_torque);

    state.motor_torque = drvSys_m_torque_commanded;


    return state;
};

drvSys_parameters drvSys_get_parameters() {
    return drvSys_parameter_config;
};

drvSys_controllerState drvSys_get_controllerState() {
    return drvSys_controller_state;
}


void _drvSys_setupDriver() {

#ifdef DRV_SYS_DEBUG
    Serial.println("DRVSYS: Setting up FOC Drive System.");
#endif // DRV_SYS_DEBUG

    drvSys_motor_encoder.init_encoder();
    drvSys_foc_controller.setup_driver();
    drvSys_foc_controller.calibrate_phase_angle(7548);


};

void drvSys_initialize() {

    /* Initial Controller Gains */
    drvSys_PID_Gains pid_gains_pos = { .K_p = 0.0, .K_i = 0.0, .K_d = 0.0 };
    drvSys_PID_Gains pid_gains_vel = { .K_p = 0.018, .K_i = 0.000, .K_d = 0.0 };

    drvSys_admittance_parameters drvSys_admittance_gains;
    drvSys_admittance_gains.virtual_spring = 0.0;
    drvSys_admittance_gains.virtual_damping = 0.0;
    drvSys_admittance_gains.virtual_inertia = 0.0;


    /* Set up Variables */
    drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_MAX_mA;
    drvSys_parameter_config.max_vel = DRVSYS_VEL_MAX;
    drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_LIMIT;
    drvSys_parameter_config.vel_pid_gains = pid_gains_vel;
    drvSys_parameter_config.pos_pid_gains = pid_gains_pos;
    drvSys_parameter_config.admittance_gains = drvSys_admittance_gains;
    drvSys_parameter_config.limit_high_deg = DRVSYS_POS_LIMIT_HIGH;
    drvSys_parameter_config.limit_low_deg = DRVSYS_POS_LIMIT_LOW;


    motor_angle_kalman_filter.init(DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US * 1e-6);

    _drvSys_setupDriver();
    _drvSys_setupInterrupts();


    /* load or calibrate Zero Angle */

    // TODO!!!!

#ifdef DRV_SYS_DEBUG
    Serial.println("DRVSYS_INFO: Loading Calibration Angle ...");
#endif 

    drvSys_motor_encoder.read_angle_raw();
    float zero_angle = drvSys_motor_encoder.get_angle_deg();
    drvSys_motor_encoder.set_zero_angle_deg(zero_angle);


    // Do something to Align Angles!!!!
    //TODO
    drvSys_controller_state.calibrated = true;


    /* initialize Torque Sensor */
    //TODO

    /* Initialize Joint Sensor */
    //TODO



    /* Create all the Static Controller Tasks */

    /* Create Controller Tasks */



        /* --- create Tasks --- */
    xTaskCreatePinnedToCore(
        _drvSys_foc_controller_task,   // function name
        "FOC_Controller_Task", // task name
        3000,      // Stack size (bytes)
        NULL,      // task parameters
        foc_prio,         // task priority
        &drvSys_foc_th,
        0 // task handle
    );

    xTaskCreatePinnedToCore(
        _drvSys_process_motor_encoder_task,   // function name
        "Process_motor_encoder_task", // task name
        2000,      // Stack size (bytes)
        NULL,      // task parameters
        9,         // task priority
        &drvSys_process_motor_encoder_th,
        0 // task handle
    );


    xTaskCreatePinnedToCore(_drvSys_torque_controller_task,
        "Torque_Controller_Task",
        1000,
        NULL,
        torque_control_prio,
        &drvSys_torque_controller_th,
        0
    );


    xTaskCreatePinnedToCore(
        _drvSys_velocity_controller_task,   // function name
        "Velocity_Controller_Task", // task name
        1000,      // Stack size (bytes)
        NULL,      // task parameters
        vel_control_prio,         // task priority
        &drvSys_vel_controller_th,
        1 // task handle
    );

    xTaskCreatePinnedToCore(_drvSys_position_controller_task,
        "Position_Controller_Task",
        1000,
        NULL,
        pos_control_prio,
        &drvSys_pos_controller_th,
        0
    );


    /* --- create Task --- */
    xTaskCreatePinnedToCore(
        _drvSys_admittance_controller_task,   // function name
        "Admittance_Controller_Task", // task name
        3000,      // Stack size (bytes)
        NULL,      // task parameters
        admittance_control_prio,         // task priority
        &drvSys_admittance_controller_th,
        0 // task handle
    );

    //suspend task until controller is started
    vTaskSuspend(drvSys_admittance_controller_th);


    //if initialization was succesful:
    drvSys_state_flag = ready;


};

void  drvSys_start_foc_processing() {

    Serial.println("DRVSYS_INFO: Start FOC Processing");

    xSemaphoreGive(drvSys_mutex_joint_position);
    xSemaphoreGive(drvSys_mutex_motor_position);
    xSemaphoreGive(drvSys_mutex_position_command);
    xSemaphoreGive(drvSys_mutex_joint_acc);
    xSemaphoreGive(drvSys_mutex_joint_vel);
    xSemaphoreGive(drvSys_mutex_motor_acc);
    xSemaphoreGive(drvSys_mutex_motor_vel);
    xSemaphoreGive(drvSys_mutex_joint_torque);
    xSemaphoreGive(drvSys_mutex_motor_commanded_torque);
    xSemaphoreGive(drvSys_mutex_torque_target);

    // Start Timer -> Starts Processing!!
    Serial.println("DRVSYS_INFO: Start Interrupt Timer");
    timerAlarmEnable(drvSys_foc_timer);


    drvSys_foc_controller.set_target_torque(0.0);

    //flag start foc controller
    drvSys_controller_state.state_flag = foc_active;
};

void drvSys_start_motion_control(drvSys_controlMode control_mode) {

    if (drvSys_controller_state.state_flag != control_active) {

        drvSys_mode = control_mode;
        drvSys_controller_state.control_mode = drvSys_mode;

        switch (drvSys_mode) {
        case cascade_position_control:
            _drvSys_setup_cascade_controller();
            break;

        case direct_torque:
            _drvSys_setup_direct_controller();
            break;

        case admittance_control:
            _drvSys_setup_admittance_controller();
            break;

        case impedance_control:
            //_drvSys_setup_impedance_controller();
            break;

        case hybrid_control:
            //_drvSys_setup_hybrid_controoller();
            break;
        default:
            _drvSys_setup_cascade_controller();
            break;
        }


        drvSys_controller_state.state_flag = control_active;
    }
    else {
        Serial.println("DRVSYS_ERROR: Can only change motion control mode when control is already stopped");
    }


};

void _drvSys_setup_cascade_controller() {
    /*Velocity Controller*/
    drvSys_velocity_controller.setSampleTime(DRVSYS_CONTROL_VEL_PERIOD_US);
    // Torque Limitation

    float torque_limit = drvSys_parameter_config.max_torque_Nm;
    drvSys_velocity_controller.setOutputLimits(torque_limit * (-1.0), torque_limit);
    drvSys_velocity_controller.setpoint = 0.0;

    drvSys_velocity_controller.Initialize();
    drvSys_velocity_controller.setMode(PID_MODE_INACTIVE);
    drvSys_velocity_controller.SetControllerDirection(PID_DIR_DIRECT);
    drvSys_velocity_controller.setErrorDeadBand(0.01);
    drvSys_velocity_controller.setOutputFilter(true, 0.5);

    _drvSys_read_vel_PID_gains_from_flash();

    drvSys_vel_ff = 0.0;


    /* Position Controller */
    drvSys_position_controller.setSampleTime(DRVSYS_CONTROL_POS_PERIOD_US);
    //Velocity Limitation
    float max_velocity = drvSys_parameter_config.max_vel;
    drvSys_position_controller.setOutputLimits(max_velocity * (-1.0), max_velocity);
    drvSys_position_controller.setpoint = 0.0;

    drvSys_position_controller.Initialize();
    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_position_controller.SetControllerDirection(PID_DIR_DIRECT);

    _drvSys_read_pos_PID_gains_from_flash();

    Serial.println("DRVSYS_INFO: Setup Cascade Position and Velocity Controller");

    drvSys_position_controller.setMode(PID_MODE_ACTIVE);
    drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);

    drvSys_torque_ff = 0.0;
    drvSys_vel_ff = 0.0;

};


void drvSys_stop_controllers() {

    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_position_controller.setMode(PID_MODE_INACTIVE);

    drvSys_controller_state.state_flag = control_inactive;

    vTaskSuspend(drvSys_admittance_controller_th);

    drvSys_set_target_motor_torque(0.0);

};

void drvSys_set_target_motor_torque(float target_torque) {

    drvSys_foc_controller.set_target_torque(target_torque);
};



void drvSys_set_target_torque(float torque) {

    float torque_limit = drvSys_parameter_config.max_torque_Nm;

    xSemaphoreTake(drvSys_mutex_torque_target, portMAX_DELAY);
    drvSys_torque_target = torque;
    if (drvSys_torque_target > torque_limit) {
        drvSys_torque_target = torque_limit;
    }
    else if (drvSys_torque_target < (-1.0) * torque_limit) {
        drvSys_torque_target = (-1.0) * torque_limit;
    }
    xSemaphoreGive(drvSys_mutex_torque_target);

};

void drvSys_set_feed_forward_torque(float torque_ff) {
    drvSys_torque_ff = torque_ff;

};

void drvSys_set_feed_forward_velocity(float vel_ff) {
    drvSys_vel_ff = vel_ff;
}

void drvSys_set_target_velocity(float vel) {


    if (vel > drvSys_parameter_config.max_vel) {
        vel = drvSys_parameter_config.max_vel;
    }
    else if (vel < (-1.0) * drvSys_parameter_config.max_vel) {
        vel = (-1.0) * drvSys_parameter_config.max_vel;
    }
    drvSys_velocity_controller.setpoint = vel;

};


void drvSys_set_target_pos(float pos) {

    if (pos > drvSys_parameter_config.limit_high_deg) {
        pos = drvSys_parameter_config.limit_high_deg;
    }
    else if (pos < drvSys_parameter_config.limit_low_deg) {
        pos = drvSys_parameter_config.limit_low_deg;
    }
    drvSys_position_controller.setpoint = pos;
};

drvSys_driveTargets drvSys_get_targets() {
    drvSys_driveTargets targets;

    targets.motor_torque_target = drvSys_foc_controller.target_torque;
    targets.pos_target = drvSys_position_controller.setpoint;
    targets.vel_target = drvSys_velocity_controller.setpoint;
    targets.ref_torque = drvSys_joint_torque_ref;

    return targets;
};

void _drvSys_setupInterrupts() {

    Serial.println("DRVSYS_INFO: Setup Control Interrupts.");
    drvSys_foc_timer = timerBegin(0, drvSys_timer_prescaler_divider, true);
    timerAttachInterrupt(drvSys_foc_timer, &_drvSys_on_foc_timer, true);
    timerAlarmWrite(drvSys_foc_timer, drvSys_timer_alarm_rate_us, true);

}

void _drvSys_set_empiric_phase_shift(float phase_shift_factor) {
    drvSys_foc_controller.set_empiric_phase_shift_factor(phase_shift_factor);
};



void IRAM_ATTR _drvSys_on_foc_timer() {
    volatile static uint64_t tickCount = 0;

    tickCount++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (tickCount % drvSys_timer_foc_ticks == 0) { // react every 200us ->5kHz
        vTaskNotifyGiveFromISR(drvSys_foc_th, &xHigherPriorityTaskWoken);
    }
    if (tickCount % drvSys_timer_encoder_process_ticks == 0) { //react every 4000us
        vTaskNotifyGiveFromISR(drvSys_process_motor_encoder_th, &xHigherPriorityTaskWoken);
    }

    if (drvSys_controller_state.state_flag == control_active) {

        if (tickCount % drvSys_timer_torque_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_torque_controller_th, &xHigherPriorityTaskWoken);
        }

        if (tickCount % drvSys_timer_vel_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_vel_controller_th, &xHigherPriorityTaskWoken);
        }
        if (tickCount % drvSys_timer_pos_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_pos_controller_th, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR();
}

void _drvSys_foc_controller_task(void* parameters) {

    uint32_t foc_thread_notification;
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;

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
            if (xSemaphoreTake(drvSys_mutex_motor_position, 10 / portTICK_PERIOD_MS) == pdTRUE) {

                /* use Kalman Filter */

                KinematicStateVector state = motor_angle_kalman_filter.estimateStates(drvSys_motor_encoder.get_angle_deg());

                drvSys_motor_position = state.pos;
                xSemaphoreGive(drvSys_mutex_motor_position);

                xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
                drvSys_motor_velocity = state.vel;
                xSemaphoreGive(drvSys_mutex_motor_vel);


                xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
                drvSys_motor_acc = state.acc;
                xSemaphoreGive(drvSys_mutex_motor_acc);



                /* Obtain Position Data from Motor Encoder */
                /*
                float motor_angle_filtered_deg = drvSys_motor_encoder.get_angle_filtered_deg();
                drvSys_motor_position = motor_angle_filtered_deg;
                xSemaphoreGive(drvSys_mutex_motor_position);

                /* Perform numerical differentiation on filtered position data */
                /*
                drvSys_differentiator_motor_pos.setInput(motor_angle_filtered_deg);
                drvSys_differentiator_motor_pos.differentiateRobustly();
                drvSys_differentiator_motor_pos.differentriateRobustlyTwice();

                /* Save Output of Differentiation to Buffers */
                /*
                xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
                drvSys_motor_velocity = drvSys_differentiator_motor_pos.getFirstDerivativeOutput();
                xSemaphoreGive(drvSys_mutex_motor_vel);

                xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
                drvSys_motor_acc = drvSys_differentiator_motor_pos.getSecondDerivativeOutput();
                xSemaphoreGive(drvSys_mutex_motor_acc);
                */



            }
            else {
                drvSys_controller_state.state_flag = error;
                /* Handle Errors of Drive System */
            }
        }

    }
};

void _drvSys_process_torque_sensor_task(void* parameters) {

    const TickType_t torque_sensor_delay = DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS / portTICK_PERIOD_MS;
    while (true) {
        //...
        // TO DO!!!!!

        vTaskDelay(torque_sensor_delay);
    }
};

void _drvSys_process_joint_sensor_task(void* parameters) {

    const TickType_t joint_sensor_delay = DRVSYS_PROCESS_JOINT_ENCODER_PERIOD_MS / portTICK_PERIOD_MS;
    while (true) {
        //....
        // TO DO!!!!!!
    }
}


void _drvSys_velocity_controller_task(void* parameters) {

    uint32_t velocity_controller_processing_thread_notification;

    while (true) {

        velocity_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (velocity_controller_processing_thread_notification) {
            if (drvSys_mode == cascade_position_control || drvSys_mode == admittance_control) {
                if (drvSys_velocity_controller.getMode() == PID_MODE_ACTIVE) {
                    xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
                    float actual_vel = drvSys_motor_velocity;
                    xSemaphoreGive(drvSys_mutex_motor_vel);

                    drvSys_velocity_controller.input = actual_vel;
                    drvSys_velocity_controller.compute();

                    /* Handle Velocity Controller Output */


                    drvSys_set_target_torque(drvSys_velocity_controller.output);


                }
            }
        }
    }
}

void _drvSys_position_controller_task(void* parameters) {
    uint32_t position_controller_processing_thread_notification;

    while (true) {

        position_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (position_controller_processing_thread_notification) {

            if (drvSys_mode == cascade_position_control || drvSys_mode == admittance_control) {
                xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
                float actual_pos = drvSys_motor_position;
                xSemaphoreGive(drvSys_mutex_motor_position);

                drvSys_position_controller.input = actual_pos;
                drvSys_position_controller.compute();

                /* Handle controller output */

                float velocity_target = drvSys_position_controller.output + drvSys_vel_ff;

                drvSys_set_target_velocity(velocity_target);


            }
        }
    }
}


void _drvSys_torque_controller_task(void* parameters) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    uint32_t torque_controller_processing_thread_notification;

    while (true) {
        torque_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (torque_controller_processing_thread_notification) {

            xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
            drvSys_m_torque_commanded = drvSys_torque_target + drvSys_torque_ff;

            drvSys_foc_controller.set_target_torque(drvSys_m_torque_commanded);
            xSemaphoreGive(drvSys_mutex_motor_commanded_torque);

        }
    };
};


void _drvSys_setup_direct_controller() {


    Serial.println("DRVSYS_INFO: Setup Direct Controller");
    drvSys_torque_ff = 0.0;


}

void _drvSys_setup_admittance_controller() {

    // Setup internal position and velocity loop
    _drvSys_setup_cascade_controller();

    _drvSys_read_admittanceGains_from_flash();
    Serial.println("DRVSYS_INFO: Setup Admittance Controller");


    vTaskResume(drvSys_admittance_controller_th);

};

void _drvSys_admittance_controller_task(void* parameters) {

    static float prev_admittance_target_pos = 0;
    static const float delta_t_admittance_control = 0.01;

    const TickType_t admittance_delay = DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {

        float virtual_spring = drvSys_parameter_config.admittance_gains.virtual_spring;
        float virtual_damping = drvSys_parameter_config.admittance_gains.virtual_damping;

        xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
        float current_joint_torque = drvSys_joint_torque;
        xSemaphoreGive(drvSys_mutex_joint_torque);

        float desired_joint_torque = drvSys_joint_torque_ref;

        float desired_velocity = drvSys_vel_ff;
        float desired_position = drvSys_pos_target;

        xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
        float current_position = drvSys_joint_position;
        xSemaphoreGive(drvSys_mutex_joint_position);


        // use relationship torque = K*error + d*error_d + torque_ref to obtain velocity target in time domain
        float vel_admittance_target = (desired_joint_torque - current_joint_torque
            + virtual_spring * (desired_position - current_position)) / virtual_damping
            + desired_velocity;

        //use first order euler integration to calculate position target
        float pos_admittance_target = prev_admittance_target_pos + delta_t_admittance_control * vel_admittance_target;


        /* Write position and velocity target to inner control loop */

        drvSys_set_feed_forward_velocity(vel_admittance_target);
        drvSys_set_target_pos(pos_admittance_target);

        vTaskDelay(admittance_delay);
    }

};






void drvSys_set_pos_PID_gains(const float Kp, const float Ki, const  float Kd, bool save) {
    // Set Position PID settings
    if (save) {
        drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

        drv_sys_preferences.putFloat("P", Kp);
        drv_sys_preferences.putFloat("I", Ki);
        drv_sys_preferences.putFloat("D", Kd);

        drv_sys_preferences.end();
    }

    // Write them to overall Drive System parameters
    drvSys_parameter_config.pos_pid_gains.K_p = Kp;
    drvSys_parameter_config.pos_pid_gains.K_i = Ki;
    drvSys_parameter_config.pos_pid_gains.K_d = Kd;

    // Write them to controller instance
    drvSys_position_controller.setTuning(Kp, Ki, Kd);
};

void drvSys_save_pos_PID_gains() {

    float* gains = drvSys_position_controller.getGains();

    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

    drv_sys_preferences.putFloat("P", gains[0]);
    drv_sys_preferences.putFloat("I", gains[1]);
    drv_sys_preferences.putFloat("D", gains[2]);

    drv_sys_preferences.end();
}
void drvSys_set_vel_PID_gains(float Kp, float Ki, float Kd, bool save) {
    // Set Position PID settings
    if (save) {
        drv_sys_preferences.begin(drvSys_velPID_saved_gains, false);

        drv_sys_preferences.putFloat("P", Kp);
        drv_sys_preferences.putFloat("I", Ki);
        drv_sys_preferences.putFloat("D", Kd);

        drv_sys_preferences.end();
    }

    // Write them to overall Drive System parameters
    drvSys_parameter_config.vel_pid_gains.K_p = Kp;
    drvSys_parameter_config.vel_pid_gains.K_i = Ki;
    drvSys_parameter_config.vel_pid_gains.K_d = Kd;

    // Write them to controller instance
    drvSys_velocity_controller.setTuning(Kp, Ki, Kd);
};

void drvSys_save_vel_PID_gains() {

    float* gains = drvSys_velocity_controller.getGains();

    drv_sys_preferences.begin(drvSys_velPID_saved_gains, false);

    drv_sys_preferences.putFloat("P", gains[0]);
    drv_sys_preferences.putFloat("I", gains[1]);
    drv_sys_preferences.putFloat("D", gains[2]);

    drv_sys_preferences.end();
};

void drvSys_set_admittance_params(float virtual_spring, float virtual_damping, float virtual_inertia, bool save) {

    if (save) {
        drv_sys_preferences.begin(drvSys_admittance_saved_gains, false);

        drv_sys_preferences.putFloat("spring", virtual_spring);
        drv_sys_preferences.putFloat("damper", virtual_damping);
        drv_sys_preferences.putFloat("inertia", virtual_inertia);

        drv_sys_preferences.end();
    }

    drvSys_parameter_config.admittance_gains.virtual_spring = virtual_spring;
    drvSys_parameter_config.admittance_gains.virtual_damping = virtual_damping;
    drvSys_parameter_config.admittance_gains.virtual_inertia = virtual_inertia;
};
void drvSys_save_admittance_params() {

    float virtual_spring = drvSys_parameter_config.admittance_gains.virtual_spring;
    float virtual_damping = drvSys_parameter_config.admittance_gains.virtual_damping;
    float virtual_inertia = drvSys_parameter_config.admittance_gains.virtual_inertia;
    drv_sys_preferences.begin(drvSys_admittance_saved_gains, false);

    drv_sys_preferences.putFloat("spring", virtual_spring);
    drv_sys_preferences.putFloat("damper", virtual_damping);
    drv_sys_preferences.putFloat("inertia", virtual_inertia);

    drv_sys_preferences.end();
};

void _drvSys_read_pos_PID_gains_from_flash() {
    /* Rea dand set position PID Settings */
    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);
    float K_pos_P = drv_sys_preferences.getFloat("P", drvSys_parameter_config.pos_pid_gains.K_p);
    float K_pos_I = drv_sys_preferences.getFloat("I", drvSys_parameter_config.pos_pid_gains.K_i);
    float K_pos_D = drv_sys_preferences.getFloat("D", drvSys_parameter_config.pos_pid_gains.K_d);
    drv_sys_preferences.end();

    // Write them to overall Drive System parameters
    drvSys_parameter_config.pos_pid_gains.K_p = K_pos_P;
    drvSys_parameter_config.pos_pid_gains.K_i = K_pos_I;
    drvSys_parameter_config.pos_pid_gains.K_d = K_pos_D;

    // Write them to controller instance
    drvSys_position_controller.setTuning(K_pos_P, K_pos_I, K_pos_D);

    Serial.println("DRVSYS_INFO: Read Position PID Gains from Flash.");
    Serial.println("DRVSYS_INFO: P = " + String(K_pos_P) + ", I = " + String(K_pos_I) + ", D = " + String(K_pos_D));

}
void _drvSys_read_vel_PID_gains_from_flash() {

    /* Read and set velocity PID Settings */
    drv_sys_preferences.begin(drvSys_velPID_saved_gains, false);
    float K_P = drv_sys_preferences.getFloat("P", drvSys_parameter_config.vel_pid_gains.K_p);
    float K_I = drv_sys_preferences.getFloat("I", drvSys_parameter_config.vel_pid_gains.K_i);
    float K_D = drv_sys_preferences.getFloat("D", drvSys_parameter_config.vel_pid_gains.K_d);
    drv_sys_preferences.end();

    // Write them to overall Drive System parameters
    drvSys_parameter_config.vel_pid_gains.K_p = K_P;
    drvSys_parameter_config.vel_pid_gains.K_i = K_I;
    drvSys_parameter_config.vel_pid_gains.K_d = K_D;

    // Write them to controller instance
    drvSys_velocity_controller.setTuning(K_P, K_I, K_D);

    Serial.println("DRVSYS_INFO: Read Velo PID Gains from Flash.");
    Serial.println("DRVSYS_INFO: P = " + String(K_P) + ", I = " + String(K_I) + ", D = " + String(K_D));

};
void _drvSys_read_admittanceGains_from_flash() {

    drv_sys_preferences.begin(drvSys_admittance_saved_gains, false);
    float virtual_spring = drv_sys_preferences.getFloat("P", drvSys_parameter_config.admittance_gains.virtual_spring);
    float virtual_damping = drv_sys_preferences.getFloat("I", drvSys_parameter_config.admittance_gains.virtual_damping);
    float virtual_inertia = drv_sys_preferences.getFloat("D", drvSys_parameter_config.admittance_gains.virtual_inertia);
    drv_sys_preferences.end();

    drvSys_parameter_config.admittance_gains.virtual_spring = virtual_spring;
    drvSys_parameter_config.admittance_gains.virtual_damping = virtual_damping;
    drvSys_parameter_config.admittance_gains.virtual_inertia = virtual_inertia;

};



void _drvSys_test_signal_task(void* parameters) {

    const TickType_t test_signal_delay = 10 / portTICK_PERIOD_MS; // 100Hz

    static int tick_counter = 0;
    static bool rectangle_toggle = false;
    static float output_sig = 0.0;
    static float sig_time = 0.0;
    const float delta_t = 0.01;


    while (drvSys_test_signal.active) {
        int ticks_half_period = (drvSys_test_signal.period / 2) / 10;

        //rectangle
        if (drvSys_test_signal.shape == 0) {


            if (tick_counter % ticks_half_period == 0) {
                rectangle_toggle = !rectangle_toggle;

                if (rectangle_toggle) {
                    output_sig = drvSys_test_signal.max;
                }
                else {
                    output_sig = drvSys_test_signal.min;
                }
            }
        }

        // triangle
        if (drvSys_test_signal.shape == 2) {
            float triangle_step_slope = (drvSys_test_signal.max - drvSys_test_signal.min) / float(ticks_half_period);

            if (tick_counter % ticks_half_period == 0) {
                rectangle_toggle = !rectangle_toggle;

                if (rectangle_toggle) {
                    triangle_step_slope = triangle_step_slope;
                }
                else {
                    triangle_step_slope = -triangle_step_slope;
                }
            }
            output_sig = output_sig + triangle_step_slope;

        }

        // sine wave
        if (drvSys_test_signal.shape == 1) {

            float frequ = 1 / drvSys_test_signal.period;

            float offset = drvSys_test_signal.max + drvSys_test_signal.min;
            float amplitude = drvSys_test_signal.max - offset;

            output_sig = offset + amplitude * sin(2 * PI * frequ * sig_time);

            sig_time = tick_counter * delta_t;

            tick_counter++;

            //output signal

            if (drvSys_test_signal.target_type == 0) {
                drvSys_set_target_pos(output_sig);
            };

            if (drvSys_test_signal.target_type == 1) {
                drvSys_set_target_velocity(output_sig);
            };
            if (drvSys_test_signal.target_type == 2) {
                drvSys_set_target_torque(output_sig);
            };


            vTaskDelay(test_signal_delay);

        }
    }

    vTaskDelete(drvSys_test_signal_th);

}


void drvSys_start_test_signal(int signal_target, int shape, float max, float min, float period) {

    drvSys_test_signal.target_type = signal_target;
    drvSys_test_signal.shape = shape;
    drvSys_test_signal.max = max;
    drvSys_test_signal.min = min;
    drvSys_test_signal.period = period;

    xTaskCreatePinnedToCore(
        _drvSys_test_signal_task,   // function name
        "output_test_signal_task", // task name
        1000,      // Stack size (bytes)
        NULL,      // task parameters
        2,         // task priority
        &drvSys_test_signal_th,
        1 // task handle
    );

    drvSys_test_signal.active = true;

    Serial.println("DRYSYS_INFO: Started Test Signal.");

};

void drvSys_stop_test_signal() {

    drvSys_test_signal.active = false;

    Serial.println("DRYSYS_INFO: Stopped Test Signal.");
}



void drvSys_calibrate_FOC() {
    Serial.println("DRVSYS: Start FOC Calibration");

    drvSys_foc_controller.calibrate_phase_angle(0);

    Serial.print("DRVSYS: Start with Phase Angle Value: ");
    Serial.println(drvSys_foc_controller.phase_null_angle);

    Serial.println("DRVSYS: Sweep Angle Area");

    bool calibration_finished = false;

    int iteration = 0;

    float trans = drvSys_motor_encoder.n_transmission;
    drvSys_motor_encoder.n_transmission = 1000000;

    float score = 0;

    float highest_score = 0;
    float best_angle = 0;

    float highest_score_has_not_changged_since = 0;

    int sweep_range = 150;
    int start_val = drvSys_foc_controller.phase_null_angle - sweep_range;

    drvSys_foc_controller.phase_null_angle = start_val;


    while (!calibration_finished) {
        drvSys_foc_controller.set_target_torque(DRVSYS_TORQUE_CONSTANT * 0.8);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        int N_samples = 300;
        float forward_vel = 0.0;
        for (int i = 0; i < N_samples; i++) {
            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            forward_vel += drvSys_motor_velocity;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        drvSys_foc_controller.set_target_torque(0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        forward_vel = abs(forward_vel / float(N_samples));


        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("Forward_vel: ");
        Serial.println(forward_vel);
        xSemaphoreGive(glob_Serial_mutex);

        drvSys_foc_controller.set_target_torque(-DRVSYS_TORQUE_CONSTANT * 0.8);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        float backward_vel = 0.0;
        for (int i = 0; i < N_samples; i++) {
            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            backward_vel += drvSys_motor_velocity;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        drvSys_foc_controller.set_target_torque(0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        backward_vel = abs(backward_vel / float(N_samples));
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("Backward_vel: ");
        Serial.println(backward_vel);
        xSemaphoreGive(glob_Serial_mutex);

        float vel_difference = abs(forward_vel - backward_vel);

        float larger_vel;
        float smaller_vel;
        if (backward_vel < forward_vel) {
            larger_vel = forward_vel;
            smaller_vel = backward_vel;
        }
        else {
            larger_vel = backward_vel;
            smaller_vel = forward_vel;
        }

        if ((forward_vel < 0.5) || (backward_vel < 0.5)) {
            score = -100;
        }
        else {

            score = (backward_vel + forward_vel) * 0.2 / vel_difference;

            score = (backward_vel + forward_vel) / larger_vel + smaller_vel / larger_vel + smaller_vel / 30;

        }
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("Score: ");
        Serial.println(score);
        Serial.print("Phase Angle value: ");
        Serial.println(drvSys_foc_controller.phase_null_angle);
        xSemaphoreGive(glob_Serial_mutex);

        if (score > highest_score) {
            highest_score = score;
            best_angle = drvSys_foc_controller.phase_null_angle;
        }


        //just sweep

        drvSys_foc_controller.phase_null_angle = drvSys_foc_controller.phase_null_angle + 1;

        if (drvSys_foc_controller.phase_null_angle > start_val + 2 * sweep_range) {
            calibration_finished = true;
        }

        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("New Phase Angle value: ");
        Serial.println(drvSys_foc_controller.phase_null_angle);
        xSemaphoreGive(glob_Serial_mutex);

        iteration++;

        if (iteration > 2 * sweep_range) {
            calibration_finished = true;
        }
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("Iteration: ");
        Serial.println(iteration);
        Serial.print("Heighest score ");
        Serial.println(highest_score);
        xSemaphoreGive(glob_Serial_mutex);

    }

    drvSys_foc_controller.phase_null_angle = best_angle;
    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.print("Final phase angle: ");
    Serial.println(drvSys_foc_controller.phase_null_angle);

    drvSys_motor_encoder.n_transmission = trans;
    Serial.println("DRVSYS: Calibration Finished");
    xSemaphoreGive(glob_Serial_mutex);

}


void drvSys_start_debug_output() {

    xTaskCreatePinnedToCore(
        _drvSys_debug_print_position_task,   // function name
        "print_position_debug_task", // task name
        2000,      // Stack size (bytes)
        NULL,      // task parameters
        2,         // task priority
        NULL,
        1 // task handle
    );
}

void _drvSys_debug_print_position_task() {


}






