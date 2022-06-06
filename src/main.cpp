#include <Arduino.h>
#include <FreeRTOS.h>
#include <drive_system.h>
#include <joint_control_asci_interface.h>

#include <FastLED.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//CAN Testing

#include <CAN.h>



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
  drvSys_start_foc_processing();

  jCtrl_asci_start_interface();

  drvSys_start_motion_control(cascade_position_control);

  //jCtrl_asci_debug_output_start(true, false, false, false, false, false);


  //drvSys_start_debug_output();
  // Calibration of FOC
  //drvSys_calibrate_FOC();



  /** Setup ASCI Interface */

  //jCtrl_asci_start_interface();

  pinMode(HALL_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL, RGB_LED_PIN>(leds, 1);

  Serial.println("Init CAN");
  CAN.setPins(CAN_RX, CAN_TX);

  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
  }
  else {
    Serial.println("Starting CAN successful!");
  }






}

void loop()
{
  /*
  leds[0] = CRGB::GreenYellow;
  FastLED.setBrightness(30);
  FastLED.show();
  vTaskDelay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  vTaskDelay(500);
  //vTaskDelay(10);
  */
  drvSys_driveState drvSys_state_data = drvSys_get_drive_state();
  xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
  Serial.print(drvSys_state_data.joint_pos);
  Serial.print(",");
  Serial.println(drvSys_state_data.joint_vel);
  /*
  Serial.print(",");
  Serial.println(drvSys_state_data.joint_acc);
  */

  xSemaphoreGive(glob_Serial_mutex);
  vTaskDelay(10);
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;

  /*
  Serial.print("Hall Sensor: ");
  Serial.println(analogRead(HALL_SENSOR_PIN));




  */

  /*
   Serial.println("Sending packet");

   CAN.beginPacket(0x12);
   CAN.write('h');
   CAN.write('e');
   CAN.write('l');
   CAN.write('l');
   CAN.write('o');
   CAN.endPacket();

   Serial.println("done");

   vTaskDelay(2000);




   // try to parse packet
   int packetSize = CAN.parsePacket();

   if (packetSize) {
     // received a packet
     Serial.print("Received ");

     if (CAN.packetExtended()) {
       Serial.print("extended ");
     }

     if (CAN.packetRtr()) {
       // Remote transmission request, packet contains no data
       Serial.print("RTR ");
     }

     Serial.print("packet with id 0x");
     Serial.print(CAN.packetId(), HEX);

     if (CAN.packetRtr()) {
       Serial.print(" and requested length ");
       Serial.println(CAN.packetDlc());
     } else {
       Serial.print(" and length ");
       Serial.println(packetSize);

       // only print packet data for non-RTR packets
       while (CAN.available()) {
         Serial.print((char)CAN.read());
       }
       Serial.println();
     }

     Serial.println();

   }*/

}