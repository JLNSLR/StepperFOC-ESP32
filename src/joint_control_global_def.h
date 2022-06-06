#ifndef JOINT_CONTROL_GLOB_DEF_H
#define JOINT_CONTROL_GLOB_DEF_H


#define JOINT_ID 0

// SPI CS PINs
#define CS_ENCODER 16
#define CS_TMC 17
#define CS_JOINT_ENCODER 4

// Function Pins
#define HALL_SENSOR_PIN 36
#define RGB_LED_PIN 33
#define FAN_PWM_PIN 15

//I2C
#define SDA 21
#define SCL 22

//TMC Related
#define TMC_DIAG1 37
#define TMC_DIAG0 36
#define TMC_STEP 25
#define TMC_DIR 26
#define TMC_DC_EN 32
#define TMC_DC0 27
#define TMC_DC_IN 14

#define R_SENSE 0.075f

//CAN
#define CAN_RX 35
#define CAN_TX 5


#include <FreeRTOS.h>

extern SemaphoreHandle_t glob_SPI_mutex;
extern SemaphoreHandle_t glob_I2C_mutex;
extern SemaphoreHandle_t glob_Serial_mutex;
extern SemaphoreHandle_t glob_CAN_mutex;

void init_global_serial_comm_mutexes();


#endif // !JOINT_CONTROL_GLOB_DEF_H