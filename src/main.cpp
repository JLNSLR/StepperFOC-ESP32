#include <Arduino.h>
#include <FreeRTOS.h>
#include <drive_system.h>

#include <FastLED.h>



// --- Global Variables --- //

CRGB leds[1];


void setup()
{
  Serial.begin(115200);
  SPI.begin();
  xSemaphoreGive(glob_SPI_mutex);
  xSemaphoreGive(glob_Serial_mutex);
  drvSys_initialize();

  Serial.println("Drive Init succesfull. Starting Drive");
  drvSys_start();

  pinMode(HALL_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL, RGB_LED_PIN>(leds, 1);



}

void loop()
{
  /*
  Serial.print("Hall Sensor: ");
  Serial.println(analogRead(HALL_SENSOR_PIN));



  */

  leds[0] = CRGB::GreenYellow;
  FastLED.setBrightness(30);
  FastLED.show();
  vTaskDelay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  vTaskDelay(500);
  //vTaskDelay(10);
}