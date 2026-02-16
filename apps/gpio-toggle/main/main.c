// Created using Shawn Hymel's excellent ESP-IDF tutorial:
// https://shawnhymel.com/2872/esp32-getting-started-with-esp-idf/
// https://github.com/ShawnHymel/course-iot-with-esp-idf/

#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

void app_main(void)
{
    // Settings
    const gpio_num_t gpio_pin = GPIO_NUM_4;
    const uint32_t sleep_time_ms = 1000;
    uint8_t gpio_state = 0;

    // Configure the GPIO
    gpio_reset_pin(gpio_pin);
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);

    // Superloop
    while (1) {

        // Toggle the GPIO
        gpio_state ^= 0x01;
        gpio_set_level(gpio_pin, gpio_state);

        // Print GPIO state
        printf("GPIO state: %d\n", gpio_state);

        // Delay
        vTaskDelay(sleep_time_ms / portTICK_PERIOD_MS); 
    }
}