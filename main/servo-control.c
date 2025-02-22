#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/ledc.h"

#define BUF_SIZE 1024  // UART buffer size

// Define Servo GPIO Pins
#define SERVO_1_GPIO 18
#define SERVO_2_GPIO 19
#define SERVO_3_GPIO 21
#define SERVO_4_GPIO 22

// Function to move servo using LEDC
void move_servo(int channel, int angle) {
    int duty = (angle * (8192 / 180)) + 410;  // Convert angle to PWM duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// UART task to read and process incoming commands
void uart_task(void *arg) {
    char data[BUF_SIZE];

    while (1) {
        int length = uart_read_bytes(UART_NUM_0, (uint8_t *)data, BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (length > 0) {
            data[length] = '\0';  // Null-terminate received data
            printf("Data received: %s\n", data);  // Debugging

            // Split data using newline as a delimiter
            char *token = strtok(data, "\n");

            while (token != NULL) {
                int servo_id, servo_angle;

                if (sscanf(token, "S%d:%d", &servo_id, &servo_angle) == 2) {
                    printf("Parsed: S%d=%d\n", servo_id, servo_angle);

                    switch (servo_id) {
                        case 0: move_servo(LEDC_CHANNEL_0, servo_angle); break;
                        case 1: move_servo(LEDC_CHANNEL_1, servo_angle); break;
                        case 2: move_servo(LEDC_CHANNEL_2, servo_angle); break;
                        case 3: move_servo(LEDC_CHANNEL_3, servo_angle); break;
                        default: printf("Invalid Servo ID!\n"); break;
                    }
                } else {
                    printf("Invalid data format: %s\n", token);
                }

                // Move to the next command in the buffer
                token = strtok(NULL, "\n");
            }
        }
        else{
            printf("no data received \n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Reduce delay for faster response
    }
}



void app_main() {
    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Initialize PWM for servos
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num = (i == 0) ? SERVO_1_GPIO :
                        (i == 1) ? SERVO_2_GPIO :
                        (i == 2) ? SERVO_3_GPIO : SERVO_4_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 410,  // Default to 0 degrees
            .hpoint = 0
        };
        ledc_channel_config(&ledc_channel);
    }

    // Start UART Task
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}