#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <joint_control_global_def.h>
#include <foc_controller_tmc2160.h>
#include <motor_encoder.h>
#include <CircularBuffer.h>
#include <AS5048A.h>
#include <TMCStepper.h>
#include <signal_processing/differentiator.h>
#include <PID/PIDController.h>
#include <drive_system_calibration.h>

#include <Preferences.h>

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

/* ##########################################################################
############## ---- Constant Drive System Parameters ----####################
############################################################################*/


/* --- Timing Constants --- */

#define DRVSYS_FOC_PERIOD_US 200 //us -> 5kHz
#define DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US 250 //us -> 4kHz
#define DRVSYS_PROCESS_MOTOR_ENCODER_FREQU 4000 //Hz
#define DRVSYS_CONTROL_TORQUE_PERIOD_US 250 // us 
#define DRVSYS_CONTROL_VEL_PERIOD_US 300 //us //600Hz
#define DRVSYS_CONTROL_VEL_FREQ 3333
#define DRVSYS_CONTROL_POS_PERIOD_US 1000 //ms //1000Hz
#define DRVSYS_CONTROL_POS_FREQ 1000

#define DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS 10
#define DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS 333
#define DRVSYS_PROCESS_JOINT_ENCODER_PERIOD_MS 1

/* --- Hardware-Timer-Constants --- */
#define DRVSYS_TIMER_PRESCALER_DIV 80 // with 80MHz Clock, makes the timer tick every 1us
#define DRVSYS_TIMER_ALARM_RATE_US 50 //generate timer alarm every 50us


/*#########################################################################
####################### --- Drive Constants --- ###########################
######################################################################## */

// need to be changed for each joint!
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6

#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0


/*--- Drive Settings --- */
#define DRVSYS_VEL_MAX 300.0 // deg/s
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800


/* DEBUG COMMAND */
#define DRV_SYS_DEBUG

/*########################################################################
################ Drive System Data Types & Enums #########################
#########################################################################*/

/* Drive System State Flag */
enum drvSys_StateFlag { not_ready, ready, foc_active, control_active, control_inactive, error };
/*Drive System Control Mode Variables */
enum drvSys_controlMode { cascade_position_control, direct_torque, admittance_control, impedance_control, hybrid_control };

extern drvSys_controlMode drvSys_mode;
extern drvSys_StateFlag drvSys_state_flag;

struct drvSys_PID_Gains {
    float K_p;
    float K_i;
    float K_d;

};

struct drvSys_admittance_parameters {
    float virtual_spring;
    float virtual_damping;
    float virtual_inertia;
};


/*#########################################################################
################# Drive System Data Structures ############################
##########################################################################*/

struct drvSys_driveState {
    float joint_pos;
    float joint_vel;
    float joint_acc;
    float joint_torque;
    float motor_torque;
};

struct drvSys_driveTargets {
    float pos_target;
    float vel_target;
    float motor_torque_target;
    float ref_torque;
};


struct drvSys_parameters {
    int max_current_mA;
    float max_torque_Nm;
    float max_vel;
    drvSys_PID_Gains vel_pid_gains;
    drvSys_PID_Gains pos_pid_gains;
    drvSys_admittance_parameters admittance_gains;
    float limit_high_deg;
    float limit_low_deg;
};

extern drvSys_parameters drvSys_parameter_config;

struct drvSys_Constants {
    const int nominal_current_mA;
    const float transmission_ratio;
    const int joint_id;
    const float motor_torque_constant;
};

struct drvSys_controllerState {
    enum drvSys_controlMode control_mode;
    enum drvSys_StateFlag state_flag;
    bool calibrated;
    bool overtemperature;
    bool temperature_warning;
    int temperature;
};


/* ##########################################################################
################### ---- Interface Functions ---- ###########################
############################################################################*/

/**
 * @brief Initializes the components of the Drive System
 *
 */
void drvSys_initialize();
/**
 * @brief starts motor encoding processing. Reads position sensors, calculates velocity and acceleration and starts FOC controller.
 *
 */
void drvSys_start_foc_processing();
/**
 * @brief starts the motion controllers depending on the control mode
 *
 */
void drvSys_start_motion_control(drvSys_controlMode = cascade_position_control);
/**
 * @brief obtain the current state of the drive system
 *
 * @return drvSys_State_t
 */
 //drvSys_State_t drvSys_get_current_state();

void drvSys_start_debug_output();

void drvSys_stop_controllers();


/* Interface Functions */

/**
 * @brief
 *
 * @return drvSys_parameters
 */
drvSys_parameters drvSys_get_parameters();
/**
 * @brief
 *
 * @return drvSys_controllerState
 */
drvSys_controllerState drvSys_get_controllerState();


/**
 * @brief sets torque target to the FOC controller - directly
 *
 * @param target_torque
 */
void drvSys_set_target_motor_torque(float target_torque);
/**
 * @brief sets torque target to the torque controller - checks for torque limits
 *
 * @param torque
 */
void drvSys_set_target_torque(float torque);

void drvSys_set_target_velocity(float vel);

void drvSys_set_target_pos(float angle);

drvSys_driveTargets drvSys_get_targets();

void drvSys_set_feed_forward_torque(float torque_ff);

void drvSys_set_feed_forward_velocity(float vel_ff);

/* Functions to change persistent parameters (PID Gains etc.) */

void drvSys_set_pos_PID_gains(float Kp, float Ki, float Kd, bool save = true);
void drvSys_save_pos_PID_gains();

void drvSys_set_vel_PID_gains(float Kp, float Ki, float Kd, bool save = true);
void drvSys_save_vel_PID_gains();

void drvSys_set_admittance_params(float virtual_spring, float virtual_damping, float virtual_inertia, bool save = true);
void drvSys_save_admittance_params();


void _drvSys_read_pos_PID_gains_from_flash();
void _drvSys_read_vel_PID_gains_from_flash();
void _drvSys_read_admittanceGains_from_flash();

void drvSys_save_angle_offset(float angle_offset);

/* ###################################################
############ Internal Drive System functions #########
###################################################### */

/**
 * @brief sets up the Motor Encoder and the FOC Controller
 *
 */
void _drvSys_setupDriver();
/**
 * @brief creates Interrupt Timers for Processing & Controllers
 *
 */
void _drvSys_setupInterrupts();


drvSys_driveState drvSys_get_drive_state();

drvSys_controllerState drvSys_get_controllerState();

drvSys_parameters drvSys_get_parameters();

drvSys_Constants drvSys_get_constants();

/* Interrupt Handler */

void IRAM_ATTR _drvSys_on_foc_timer();

/* ############################
########### RTOS TASKS ########
###############################*/

/* FOC-Control */
void _drvSys_process_motor_encoder_task(void* parameters);

void _drvSys_foc_controller_task(void* parameters);

/* Process Sensor Tasks */
void _drvSys_process_torque_sensor_task(void* parameters);

void _drvSys_process_joint_sensor_task(void* parameters);

void _drvSys_set_empiric_phase_shift(float phase_shift_factor);


/* --- RTOS Controller Tasks --- */
void _drvSys_position_controller_task(void* parameters);

void _drvSys_velocity_controller_task(void* parameters);

void _drvSys_torque_controller_task(void* parameters);

void _drvSys_admittance_controller_task(void* parameters);


/* Debug & Test Tasks */
void _drvSys_debug_print_position_task(void* parameters);

void _drvSys_read_serial_commands_task(void* parameters);

void _drvSys_test_signal_task(void* parameters);

/**
 * @brief Starts a test signal task for debugging and controller tuning
 *
 * @param signal_target 0 - pos, 1 - vel, 2 - m_torque, 3 - output_torque
 * @param shape 0 - square, 1 - sine wave, 2 - ramp function
 * @param max maximum value (deg, deg/s, Nm)
 * @param min minimum value (deg, deg/s, Nm)
 * @param period (ms)
 */
void drvSys_start_test_signal(int signal_target, int shape, float max, float min, float period);

void drvSys_stop_test_signal();

/* internal Funvtions */

void _drvSys_setup_cascade_controller();

void _drvSys_setup_direct_controller();

void _drvSys_setup_admittance_controller();

//void _drvSys_setup_impedance_controller();

//void _drvSys_setup_hybrid_controoller();

void drvSys_calibrate_FOC();








#endif //DRIVE_SYSTEM_H