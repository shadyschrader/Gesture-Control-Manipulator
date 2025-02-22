#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define SERVO_1_GPIO 18
#define SERVO_2_GPIO 19
#define SERVO_3_GPIO 21
#define SERVO_4_GPIO 22

#define MIN_PULSEWIDTH 1000  // 1ms (0 degrees)
#define MAX_PULSEWIDTH 2000  // 2ms (180 degrees)
#define SWEEP_DELAY 20  // Delay in ms per step

mcpwm_cmpr_handle_t comparator[4];  // Store comparator handles
mcpwm_gen_handle_t generator[4];    // Store generator handles

void set_servo_angle(int servo, int angle) {
    int pulse_width = MIN_PULSEWIDTH + (angle * (MAX_PULSEWIDTH - MIN_PULSEWIDTH) / 180);
    mcpwm_comparator_set_compare_value(comparator[servo], pulse_width);
}

void servo_task(void *arg) {
    while (1) {
        for (int angle = 0; angle <= 180; angle += 5) {
            for (int i = 0; i < 4; i++) {
                set_servo_angle(i, angle);
            }
            vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY));
        }
        for (int angle = 180; angle >= 0; angle -= 5) {
            for (int i = 0; i < 4; i++) {
                set_servo_angle(i, angle);
            }
            vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY));
        }
    }
}

void configure_servo(int servo, int gpio, mcpwm_oper_handle_t oper) {
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(oper, &comparator_config, &comparator[servo]);

    mcpwm_generator_config_t generator_config = {.gen_gpio_num = gpio};
    mcpwm_new_generator(oper, &generator_config, &generator[servo]);

    mcpwm_comparator_set_compare_value(comparator[servo], 1500);  // Center position
    mcpwm_generator_set_action_on_timer_event(generator[servo],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator[servo],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[servo], MCPWM_GEN_ACTION_LOW));
}

void app_main(void) {
    mcpwm_timer_handle_t timer[2];  // Two timers for two operators
    mcpwm_oper_handle_t oper[2];    // Two operators (one for each set of 2 servos)

    for (int i = 0; i < 2; i++) {
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,  // 1 MHz resolution (1us per tick)
            .period_ticks = 20000,  // 20ms period (50Hz PWM)
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        mcpwm_new_timer(&timer_config, &timer[i]);

        mcpwm_operator_config_t operator_config = {.group_id = 0};
        mcpwm_new_operator(&operator_config, &oper[i]);
        mcpwm_operator_connect_timer(oper[i], timer[i]);
    }

    configure_servo(0, SERVO_1_GPIO, oper[0]);
    configure_servo(1, SERVO_2_GPIO, oper[0]);
    configure_servo(2, SERVO_3_GPIO, oper[1]);
    configure_servo(3, SERVO_4_GPIO, oper[1]);

    mcpwm_timer_enable(timer[0]);
    mcpwm_timer_enable(timer[1]);
    mcpwm_timer_start_stop(timer[0], MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_start_stop(timer[1], MCPWM_TIMER_START_NO_STOP);

    xTaskCreate(servo_task, "servo_task", 2048, NULL, 1, NULL);
}
