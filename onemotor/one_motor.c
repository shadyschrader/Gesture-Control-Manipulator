#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define SERVO_GPIO 22 // Change this to your servo's PWM pin
#define MIN_PULSEWIDTH 1000  // 1 ms (0 degrees)
#define MAX_PULSEWIDTH 2000  // 2 ms (180 degrees)
#define SWEEP_DELAY 20  // Delay in ms per step

mcpwm_cmpr_handle_t comparator;

void set_servo_angle(int angle) {
    int pulse_width = MIN_PULSEWIDTH + (angle * (MAX_PULSEWIDTH - MIN_PULSEWIDTH) / 180);
    mcpwm_comparator_set_compare_value(comparator, pulse_width);
}

void app_main(void) {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 MHz, 1us per tick
        .period_ticks = 20000,  // 20ms period (50Hz PWM)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    mcpwm_new_operator(&operator_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(oper, &comparator_config, &comparator);

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {.gen_gpio_num = SERVO_GPIO};
    mcpwm_new_generator(oper, &generator_config, &generator);

    mcpwm_comparator_set_compare_value(comparator, 1500);  // Start at center position
    mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    while (1) {
        for (int angle = 0; angle <= 180; angle += 5) {
            set_servo_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY));
        }
        for (int angle = 180; angle >= 0; angle -= 5) {
            set_servo_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY));
        }
    }
}
