/*
*DS-S009 Sync Read Example
*同步读取位置示例
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

uint8_t ID[] = {1, 2};
uint8_t rxPacket[4];

extern "C" void app_main(void)
  {
      ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
      vTaskDelay(pdMS_TO_TICKS(1000));

      while (1) {
          // 广播同步读：ID=1,2 从寄存器 0x38 各返回 4 字节 (位置+速度)
          ds_servo.syncReadPacketTx(ID, sizeof(ID), DS_REG_CUR_POS, 4);

          for (uint8_t i = 0; i < sizeof(ID); i++) {
              int16_t len = ds_servo.syncReadPacketRx(ID[i], rxPacket, sizeof(rxPacket));
              if (len < 2) {
                  printf("ID: %d sync read error!\n", ID[i]);
                  continue;
              }
              int16_t Position = ds_servo.syncReadRxPacketToWord(rxPacket, 0);
              int16_t Speed    = ds_servo.syncReadRxPacketToWord(rxPacket, 2);
              printf("ID: %d Position: %d Speed: %d\n", ID[i], Position, Speed);
          }
          vTaskDelay(pdMS_TO_TICKS(100));
      }
  }