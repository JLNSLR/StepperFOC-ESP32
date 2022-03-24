#include <joint_control_global_def.h>

SemaphoreHandle_t glob_SPI_mutex = xSemaphoreCreateBinary();
SemaphoreHandle_t glob_I2C_mutex = xSemaphoreCreateBinary();
SemaphoreHandle_t glob_CAN_mutex = xSemaphoreCreateBinary();

SemaphoreHandle_t glob_Serial_mutex = xSemaphoreCreateBinary();


void init_global_serial_comm_mutexes() {
    xSemaphoreGive(glob_SPI_mutex);
    xSemaphoreGive(glob_I2C_mutex);
    xSemaphoreGive(glob_CAN_mutex);
}