/*
*DS-S009 Sync Write Position Example
*同步写入位置示例
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
      ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
      vTaskDelay(pdMS_TO_TICKS(1000));

      DSSyncWriteData data[2];

      while (1) {
          // 舵机1 → 位置 1024, 1000ms
          data[0].id = 1;
          data[0].position = 1024;
          data[0].time_ms = 1000;

          // 舵机2 → 位置 3072, 1000ms (同时启动)
          data[1].id = 2;
          data[1].position = 3072;
          data[1].time_ms = 1000;

          ds_servo.syncWritePosition(data, 2);
          printf("Sync move: ID1→1024, ID2→3072\n");
          vTaskDelay(pdMS_TO_TICKS(2000));

          // 反向
          data[0].position = 3072;
          data[1].position = 1024;
          ds_servo.syncWritePosition(data, 2);
          printf("Sync move: ID1→3072, ID2→1024\n");
          vTaskDelay(pdMS_TO_TICKS(2000));
      }
  }