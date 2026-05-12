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

      printf("Programming EPROM...\n");

      // 解锁 EEPROM: 寄存器 0x30 写 0x00
      uint8_t unlock[2] = {0x30, 0x00};
      ds_servo.sendPacket(1, DS_CMD_WRITE, unlock, 2);
      vTaskDelay(pdMS_TO_TICKS(20));

      // 修改 ID: 寄存器 0x05 写 2
      uint8_t id_params[2] = {0x05, 2};
      ds_servo.sendPacket(1, DS_CMD_WRITE, id_params, 2);
      vTaskDelay(pdMS_TO_TICKS(20));

      // 上锁 EEPROM: 寄存器 0x30 写 0x01
      uint8_t lock[2] = {0x30, 0x01};
      ds_servo.sendPacket(2, DS_CMD_WRITE, lock, 2);  // 注意: 这里 ID 已是 2
      printf("EPROM programmed! ID=1 → ID=2\n");

      while (1) {
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
  }