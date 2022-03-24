#ifndef JOINT_CONTROL_GLOB_DEF_H
#define JOINT_CONTROL_GLOB_DEF_H

#define EN_PIN 15  // Enable
#define DIR_PIN 4  // Direction
#define STEP_PIN 2 // Step

#define R_SENSE 0.075f // Match to your driver                         \
                       // SilentStepStick series use 0.11              \
                       // UltiMachine Einsy and Archim2 boards use 0.2 \
                       // Panucatt BSD2660 uses 0.1                    \
                       // Watterott TMC5160 uses 0.075

#define CS_ENCODER 16
#define CS_TMC 17

#include <FreeRTOS.h>

extern SemaphoreHandle_t glob_SPI_mutex;
extern SemaphoreHandle_t glob_I2C_mutex;
extern SemaphoreHandle_t glob_Serial_mutex;
extern SemaphoreHandle_t glob_CAN_mutex;

void init_global_serial_comm_mutexes();


#endif // !JOINT_CONTROL_GLOB_DEF_H