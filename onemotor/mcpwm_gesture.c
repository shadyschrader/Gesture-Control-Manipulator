#include <stdio.h>
#include <string.h>
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_GPIO 5        // Servo signal pin (change if needed)
#define UART_NUM UART_NUM_0 // UART0 (default serial port)
#define BUF_SIZE 128        // UART buffer size

#define SERVO_MIN_PULSEWIDTH 500   // Minimum pulse width in microseconds (0°)
#define SERVO_MAX_PULSEWIDTH 2500  // Maximum pulse width in microseconds (180°)
#define SERVO_FREQ 50              // 50Hz frequency (20ms period)

// MCPWM Configuration
mcpwm_cmpr_handle_t comparator;
mcpwm_gen_handle_t generator;

void set_servo_angle(int angle) {
    int pulse_width = SERVO_MIN_PULSEWIDTH + (angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / 180);
    mcpwm_comparator_set_compare_value(comparator, pulse_width);
    ESP_LOGI("SERVO", "Angle: %d°, Pulse Width: %d us", angle, pulse_width);
}

void app_main() {
    // UART Configuration
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // TX=GPIO1, RX=GPIO3

    // MCPWM Configuration for Servo
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_oper_handle_t operator = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 MHz resolution
        .period_ticks = 20000,     // 20ms period (50Hz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_operator_config_t operator_config = {.group_id = 0};
    mcpwm_new_operator(&operator_config, &operator);

    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operator, &comparator_config, &comparator);

    mcpwm_generator_config_t generator_config = {.gen_gpio_num = SERVO_GPIO};
    mcpwm_new_generator(operator, &generator_config, &generator);

    mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    char data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the string
            int angle = atoi(data); // Convert received data to integer
            if (angle >= 0 && angle <= 180) {
                set_servo_angle(angle);
            }
        }
    }
}
