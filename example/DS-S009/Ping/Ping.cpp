/*
*DS-S009 Ping Example
*Ping示例
*ESP-IDF v5.4.4
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DSServo.h"

#define SERVO_UART_NUM UART_NUM_1
#define SERVO_BAUDRATE 1000000
#define SERVO_TX_PIN 36
#define SERVO_RX_PIN 32

DSServo ds_servo;

extern "C" void app_main(void)
{
    //Intialize the Serial Port
    ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the serial port to stabilize

      while (1) {
          if (ds_servo.ping(1) == ESP_OK) {
              printf("Servo ID: 1 online\n");
              vTaskDelay(pdMS_TO_TICKS(100));
          } else {
              printf("Ping servo ID=1 error!\n");
              vTaskDelay(pdMS_TO_TICKS(2000));
          }
      }
}