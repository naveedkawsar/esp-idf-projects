#include <stdio.h> 
#include <string.h> 
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/uart.h" 
#include "esp_log.h" 

// UART configuration 
#define UART_NUM        (UART_NUM_0 )
#define UART_RXD_PIN    (3)
#define UART_TXD_PIN    (1)
#define UART_BUFFER_SIZE (256)

void uart_task(void *pvParameters) 
{
    uint8_t rx_buffer[UART_BUFFER_SIZE]; 
    int len;
    char *current_taskname = pcTaskGetName(NULL);

    while (1) {
        memset(rx_buffer, 0, sizeof(rx_buffer));

        // Read data from UART with timeout 
        len = uart_read_bytes(UART_NUM, rx_buffer, UART_BUFFER_SIZE, 10 / portTICK_PERIOD_MS); 

        if (len > 0) { 
            // Echo back the received data
            //uart_write_bytes(UART_NUM, (char*)rx_buffer, len); 
            ESP_LOGI(current_taskname, "Read %d bytes: '%s'", len, rx_buffer);

            // Optional: Print raw bytes to console for debugging
            ESP_LOG_BUFFER_HEXDUMP(current_taskname, rx_buffer, len, ESP_LOG_INFO);
        } 

        // Small delay to prevent excessive CPU usage 
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    } 
    vTaskDelete(NULL); 
} 


void app_main(void) 
{ 
    esp_err_t esp_ret;
    char *current_taskname = pcTaskGetName(NULL);

    // Configure UART 
    uart_config_t uart_config = { 
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT,
    }; 

    // Install UART driver 
    esp_ret = uart_param_config(UART_NUM, &uart_config);
    ESP_ERROR_CHECK(esp_ret);
    esp_ret = uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(esp_ret);
    esp_ret = uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(esp_ret);

    ESP_LOGI(current_taskname, "USB echo server started"); 

    // Start the UART task 
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL); 
}