// Created using Shawn Hymel's excellent ESP-IDF tutorial:
// https://shawnhymel.com/2872/esp32-getting-started-with-esp-idf/
// https://github.com/ShawnHymel/course-iot-with-esp-idf/
// See also Low Level Learning's in-depth Getting Started video:
// https://www.youtube.com/watch?v=dOVjb2wXI84

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define SLEEP_TIME_MS (1000u)

void app_main(void)
{
    esp_err_t esp_ret;
    uint8_t gpio_state = 0;

    char *current_taskname = pcTaskGetName(NULL);

    // Configure the GPIO
    esp_ret = gpio_reset_pin(GPIO_NUM_4);
    ESP_ERROR_CHECK(esp_ret);

    esp_ret = gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(esp_ret);

    // Superloop
    while (1) {

        // Toggle the GPIO
        gpio_state ^= 0x01;
        esp_ret = gpio_set_level(GPIO_NUM_4, gpio_state);
        ESP_ERROR_CHECK(esp_ret);

        // Print GPIO state
        ESP_LOGI(current_taskname, "GPIO state: %d\n", gpio_state);

        // Delay
        vTaskDelay(SLEEP_TIME_MS / portTICK_PERIOD_MS); 
    }
}