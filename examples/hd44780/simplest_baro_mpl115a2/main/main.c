#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <pcf8574.h>
#include <mpl115a2.h>
#include <string.h>

static i2c_dev_t pcf8574;

static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

void lcd_test(void *pvParameters)
{
    // Fill LCD descriptor
    hd44780_t lcd = {
        .write_cb = write_lcd_data, // use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 4,
        .pins = {
            .rs = 0,
            .e  = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3
        }
    };

    // Prepare PCF8574
    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, CONFIG_EXAMPLE_I2C_PCF8574_ADDRESS, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Prepare MPL115A2
    mpl115a2_t dev;
    memset(&dev, 0, sizeof(mpl115a2_t));
    ESP_ERROR_CHECK(mpl115a2_init_desc(&dev, MPL115A2_I2C_ADDRESS_0, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(mpl115a2_init(&dev));

    // Init screen, switch backlight on
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_switch_backlight(&lcd, true);

    char s[21];
    uint32_t count = 0;
    float pressure, temperature;

    while (1)
    {
        // read sensor
        if (mpl115a2_read_float(&dev, &temperature, &pressure) != ESP_OK)
        {
            hd44780_clear(&lcd);
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Failed!");
        }
        
        else
        {
            // print data
            snprintf(s, sizeof(s), "t: %.01f C / %.01f F", temperature, temperature*1.8+32);
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, s);

            snprintf(s, sizeof(s), "P: %.01f kPa  ", pressure / 1000);
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, s);

            snprintf(s, sizeof(s), "%lu", count++);
            hd44780_gotoxy(&lcd, 0, 2);
            hd44780_puts(&lcd, s);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_test, "lcd_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}

