#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//CODE FOR MOVING ONLY ONE MOTOR

#define SERVO_MIN_PULSEWIDTH 500
#define SERVO_MAX_PULSEWIDTH 2500
#define SERVO_FREQUENCY 50

#define SERVO1_GPIO 18
#define SERVO2_GPIO 19
#define SERVO3_GPIO 21
#define SERVO4_GPIO 22

#define UART_NUM UART_NUM_0
#define UART_TX_PIN 17
#define UART_RX_PIN 16 
#define BUF_SIZE 1024


void servos(int gpio , ledc_channel_t channel)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit resolution
        .freq_hz = SERVO_FREQUENCY,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  // Start with 0 duty cycle
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
}

//we need a function to convert the angle to duty cycle
uint32_t angle_to_duty(int angle)
{
    int duty = (angle*(SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH)/180) +  SERVO_MIN_PULSEWIDTH;
    return (duty*(1 << 13))/20000;
}

void move_servo(ledc_channel_t channel, int angle)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, angle_to_duty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);   
}

void app_main(void)
{
    servos(SERVO1_GPIO, LEDC_CHANNEL_0);
    servos(SERVO2_GPIO, LEDC_CHANNEL_1);
    servos(SERVO3_GPIO, LEDC_CHANNEL_2);
    servos(SERVO4_GPIO, LEDC_CHANNEL_3);

    while (1) {
        printf("Moving Servos...\n");
        for (int angle = 0; angle <= 180; angle += 30) {
            move_servo(LEDC_CHANNEL_0, angle);
            move_servo(LEDC_CHANNEL_1, 180 - angle);
            move_servo(LEDC_CHANNEL_2, angle);
            move_servo(LEDC_CHANNEL_3, 180 - angle);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}