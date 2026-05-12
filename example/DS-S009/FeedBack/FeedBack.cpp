/*
*DS-S009 FeedBack Example
*反馈示例
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
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the serial port

    while(1)
    {
        ds_servo.FeedBack(1); // Get feedback from servo with ID 1 (you can change the ID to read other servos)
        if(!ds_servo.getLastError()){
            int Pos = ds_servo.ReadPos(1);
            int Speed = ds_servo.ReadSpeed(1);
            int Load = ds_servo.ReadLoad(1);
            int Voltage = ds_servo.ReadVoltage(1);
            int Temper = ds_servo.ReadTemper(1);
            int Move = ds_servo.ReadMove(1);
            int Current = ds_servo.ReadCurrent(1);

            printf("Position: %d\n", Pos);
            printf("Speed: %d\n", Speed);
            printf("Load: %d\n", Load);
            printf("Voltage: %d\n", Voltage);
            printf("Temperature: %d\n", Temper);
            printf("Moving: %d\n", Move);
            printf("Current: %d mA\n", Current);
            printf("---\n");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before the next feedback read
        }else{
            printf("Failed to read feedback from servo.\n");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before retrying
        }
    }
}