/*
*DS-S009 Write Speed Example
*恒速模式示例
*ESP-IDF v5.4.4
*/

  #include <stdio.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "DSServo.h"

  #define SERVO_UART_NUM    UART_NUM_1
  #define SERVO_BAUDRATE    1000000
  #define SERVO_TX_PIN      36
  #define SERVO_RX_PIN      32

  DSServo ds_servo;

  extern "C" void app_main(void)
  {
      ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
      vTaskDelay(pdMS_TO_TICKS(1000));

      ds_servo.WheelMode(1);  // 舵机 ID=1 切换至恒速模式

      while (1) {
          // 正向旋转 (速度 60, 加速度 50)
          ds_servo.WriteSpe(1, 60, 50);
          vTaskDelay(pdMS_TO_TICKS(5000));

          // 停止
          ds_servo.WriteSpe(1, 0, 50);
          vTaskDelay(pdMS_TO_TICKS(2000));

          // 反向旋转 (速度 -60, 加速度 50)
          ds_servo.WriteSpe(1, -60, 50);
          vTaskDelay(pdMS_TO_TICKS(5000));

          // 停止
          ds_servo.WriteSpe(1, 0, 50);
          vTaskDelay(pdMS_TO_TICKS(2000));
      }
  }