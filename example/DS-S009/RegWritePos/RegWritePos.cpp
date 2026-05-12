/*
*DS-S009 Register Write Position Example
*异步写位置指令
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

    while(1){
        // Write position 2048 (135 degree) to servo with ID 1, with a movement time of 500 ms
        ds_servo.setPosition(1, 2048, 500);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for the movement to complete

        // Write position 1024 (67.5 degree) to servo with ID 1, with a movement time of 500 ms
        ds_servo.setPosition(1, 1024, 500);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for the movement to complete
    }
}