#include <Arduino.h>
#include <FreeRTOS.h>
#include <drive_system.h>



// --- Global Variables --- //


void setup()
{
  Serial.begin(115200);
  SPI.begin();
  xSemaphoreGive(glob_SPI_mutex);
  xSemaphoreGive(glob_Serial_mutex);
  drvSys_initialize();

  Serial.println("Drive Init succesfull. Starting Drive");
  drvSys_start();



}

void loop()
{

}