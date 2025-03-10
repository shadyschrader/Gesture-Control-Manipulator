#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_1  // Use UART1
#define BUF_SIZE 128

void app_main() {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Apply configuration
    uart_param_config(UART_NUM, &uart_config);
    
    // Set TX and RX pins (Adjust as needed)
    uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    esp_err_t err = uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        printf("Failed to install UART driver: %d\n", err);
        return;
    }

    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';  // Null-terminate the received data
            printf("Received: %s\n", data);
        } else {
            printf("No data received\n");
        }

        vTaskDelay(500 / pdMS_TO_TICKS(100));  // Delay to prevent spamming
    }
}
