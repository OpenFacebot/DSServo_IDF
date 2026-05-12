/*
*DS-S009 Calibration Example
*中位校准
*ESP-IDF v5.4.4
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "DSServo.h"

#deine SERVO_UART_NUM UART_NUM_1
#define SERVO_BAUDRATE 1000000
#define SERVO_TX_PIN 36
#define SERVO_RX_PIN 32

DSServo ds_servo;

extern "C" void app_main(void)
{
    //Intialize the Serial Port
    ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the serial port to stabilize

    printf("Starting calibration...\n");
    ds_servo.calibrateOffset(1); // Calibarte servo with ID 1 (you can change the ID to calibrate other servos)
     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the calibration to complete
    printf("Calibration completed.\n");

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000)); // Keep the task alive
    }
}