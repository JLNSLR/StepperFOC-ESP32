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

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

/* ##########################################################################
############## ---- Constant Drive System Parameters ----####################
############################################################################*/


extern uint64_t f_count;
extern uint64_t e_count;

/* --- Timing Constants --- */

#define DRVSYS_FOC_PERIOD_US 250 //us -> 5kHz
#define DRVSYS_PROCESS_MOTOR_ENCODER_PERIOD_US 250 //us -> 4kHz
#define DRVSYS_CONTROL_ACC_PERIOD_US 500 //us //2kHz
#define DRVSYS_CONTROL_VEL_PERIOD_US 1666 //us //600Hz
#define DRVSYS_CONTROL_POS_PERIOD_MS 2 //ms //200Hz

/* --- Hardware-Timer-Constants --- */
#define DRVSYS_TIMER_PRESCALER_DIV 80 // with 80MHz Clock, makes the timer tick every 1us
#define DRVSYS_TIMER_ALARM_RATE_US 50 //generate timer alarm every 50us


/*--- Drive Constants --- */
#define DRVSYS_PHASE_CURRENT_MAX_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 2
#define DRVSYS_TORQUE_CONSTANT 0.45


/* DEBUG COMMAND */
#define DRV_SYS_DEBUG

/* Drive System State Flag */
enum drvSys_StateFlag { not_ready, ready, active, inactive, error, eroor_signal_processing, error_control };

struct drvSys_State_t {
    float pos;
    float vel;
    float acc;
    float motor_torque;
    float output_torque;
};


/* ##########################################################################
################### ---- Interface Functions ---- ###########################
############################################################################*/

void drvSys_initialize();
void drvSys_start();
void drvSys_set_target_motor_torque(float target_torque);
drvSys_State_t drvSys_get_current_state();





/* --- Internal Drive System functions --- */

void drvSys_setupDriver();
void drvSys_setupInterrupts();

/* Interrupt Handler */

void IRAM_ATTR drvSys_on_foc_timer();

/* RTOS TASKS */

void _drvSys_process_motor_encoder_task(void* parameters);

void _drvSys_foc_controller_task(void* parameters);

void _drvSys_position_controller_task(void* parameters);

void _drvSys_velocity_controller_task(void* parameters);

void _drvSys_acceleration_controller_task(void* parameters);

void _drvSys_process_torque_sensor_task(void* parameters);

void _drvSys_torque_controller_task(void* parameters);

void _drvSys_debug_print_position_task(void* parameters);













#endif //DRIVE_SYSTEM_H