#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/ledc.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

#define SERVO1_GPIO 18  // First motor (change as needed)
#define SERVO2_GPIO 19  // Second motor (change as needed)

#define SERVO1_CHANNEL LEDC_CHANNEL_0
#define SERVO2_CHANNEL LEDC_CHANNEL_1

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_MIN_PULSEWIDTH 500   // 500 µs (0°)
#define SERVO_MAX_PULSEWIDTH 2500  // 2500 µs (180°)
#define SERVO_MAX_DEGREE 180       // Max servo rotation



void set_servo_angle(int angle, int channel) {
    int duty = SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle / SERVO_MAX_DEGREE);
    duty = (duty * 8192) / 20000;  // Convert µs to duty cycle (for 20ms period)
    
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}


void set_servo_angle_smooth(int target_angle1, int target_angle2, int delay_ms) {
    static int current_angle1 = 90;
    static int current_angle2 = 90;

    for (int angle = 0; angle <= abs(target_angle1 - current_angle1) || angle <= abs(target_angle2 - current_angle2); angle++) {
        if (target_angle1 > current_angle1) current_angle1++;
        else if (target_angle1 < current_angle1) current_angle1--;

        if (target_angle2 > current_angle2) current_angle2++;
        else if (target_angle2 < current_angle2) current_angle2--;

        set_servo_angle(current_angle1, SERVO1_CHANNEL);
        set_servo_angle(current_angle2, SERVO2_CHANNEL);
        vTaskDelay(delay_ms / pdMS_TO_TICKS(100));
    }
}


void process_gesture(const char *gesture) {
    if (strcmp(gesture, "GRAB") == 0) {
        set_servo_angle_smooth(0, 180, 20);  // Servo1 = 0°, Servo2 = 180°
    } else if (strcmp(gesture, "RELEASE") == 0) {
        set_servo_angle_smooth(180, 0, 20);  // Servo1 = 180°, Servo2 = 0°
    } else if (strcmp(gesture, "PINCH") == 0) {
        set_servo_angle_smooth(90, 90, 20);  // Both to 90°
    }
}


void setup_servos() {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config1 = {
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = SERVO1_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config1);

    ledc_channel_config_t channel_config2 = {
        .gpio_num = SERVO2_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = SERVO2_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config2);
}


void app_main() {
    setup_servos();  // Initialize both servos

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t data[BUF_SIZE];
    char *token;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            token = strtok((char *)data, "\n");
            while (token != NULL) {
                printf("Received: %s\n", token);
                process_gesture(token);
                token = strtok(NULL, "\n");
            }
        }

        vTaskDelay(100 / pdMS_TO_TICKS(100));
    }
}
