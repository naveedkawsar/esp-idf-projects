// Based on Shawn Hymel's ESP-IDF I2C tutorial and Espressif's basic I2C example:
// https://shawnhymel.com/2954/esp32-how-to-use-i2c-with-esp-idf/
// https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_basic
// For Espressif's comprehensive I2C API, see:
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html 

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define I2C_PORT_NUM_0          (0)
#define I2C_MASTER_SDA_IO       (4)
#define I2C_MASTER_SCL_IO       (5)
#define I2C_FREQ_100KHZ         (100000u)
#define I2C_FREQ_400KHZ         (400000u)
#define I2C_MASTER_TIMEOUT_MS   (1000) // -1 means wait forever

#define TMP10X_ADDR_A0_0        (0x48)
#define TMP10X_ADDR_A0_1        (0x49)
#define TMP10X_TEMP_REG         (0x00)
#define TMP10x_CONFIG_REG       (0x01)


static const uint32_t sleep_time_ms = 1000;
static const char *TAG = "i2c-master-demo";

static esp_err_t i2c_dev_register_read(
                                        i2c_master_dev_handle_t dev_handle,
                                        uint8_t *reg_addr,
                                        uint8_t *data,
                                        size_t len
                                        )
{
    return i2c_master_transmit_receive(dev_handle, reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}


void app_main(void)
{
    esp_err_t esp_ret;
    i2c_master_bus_handle_t i2c0_bus_handle;
    i2c_master_dev_handle_t tmp10x_handle;
    uint8_t reg;
    uint8_t rx[2];
    int16_t temperature;

    // Set I2C bus configuration
    i2c_master_bus_config_t i2c0_config = {
        .i2c_port = (i2c_port_num_t)I2C_PORT_NUM_0, // -1 for auto-select
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Initialize the I2C bus
    esp_ret = i2c_new_master_bus(&i2c0_config, &i2c0_bus_handle);
    ESP_ERROR_CHECK(esp_ret);

    
    // Set I2C device configuration
    i2c_device_config_t tmp10x_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = (uint16_t)TMP10X_ADDR_A0_0,
        .scl_speed_hz = (uint32_t)I2C_FREQ_100KHZ,
    };

    // Initialize the TMP10x I2C device on the bus
    esp_ret = i2c_master_bus_add_device(i2c0_bus_handle, &tmp10x_config, &tmp10x_handle);
    ESP_ERROR_CHECK(esp_ret);

    // Superloop
    while (1)
    {
        // Delay
        vTaskDelay(sleep_time_ms / portTICK_PERIOD_MS);

        reg = (uint8_t)TMP10X_TEMP_REG;
        rx[0] = 0;
        rx[1] = 0;

        // Read temperature
        esp_ret = i2c_dev_register_read(tmp10x_handle, &reg, rx, sizeof(rx));
        ESP_ERROR_CHECK(esp_ret);
      
        // Convert data to temperature (deg C)
        temperature = (rx[0] << 8) | rx[1];
        temperature >>= 4;
        temperature *= 0.0625;

        // Print temperature
        ESP_LOGI(TAG, "Temperature: %d deg C\r\n", temperature);
    }

    // Should never get here
    esp_ret = i2c_master_bus_rm_device(tmp10x_handle);
    ESP_ERROR_CHECK(esp_ret);
    esp_ret= i2c_del_master_bus(i2c0_bus_handle);
    ESP_ERROR_CHECK(esp_ret);

}