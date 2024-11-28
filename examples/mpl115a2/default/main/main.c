#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <mpl115a2.h>
#include <string.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void mpl115a2_test(void *pvParameters)
{
    mpl115a2_params_t params;
    mpl115a2_init_default_params(&params);
    mpl115a2_t dev;
    memset(&dev, 0, sizeof(mpl115a2_t));

    ESP_ERROR_CHECK(mpl115a2_init_desc(&dev, MPL115A2_I2C_ADDRESS_0, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(mpl115a2_init(&dev, &params));

    float pressure, temperature;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mpl115a2_read_float(&dev, &temperature, &pressure) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(mpl115a2_test, "mpl115a2_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

