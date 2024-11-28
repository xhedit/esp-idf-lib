/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 sheinz <https://github.com/sheinz>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file mpl115a2.c
 *
 * ESP-IDF driver for MPL115A2 digital pressure/temp sensor
 *
 * Adopted from bmp2809 driver
 *
 * Copyright (c) 2016 sheinz <https://github.com/sheinz>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2023 xhedit <https://github.com/xhedit>
 * 
 * MIT Licensed as described in the file LICENSE
 */

#include "mpl115a2.h"
#include <inttypes.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 100000 // Max 1MHz for esp-idf

static const char *TAG = "mpl115a2";

#define MPL115A2_REG_PRESSURE_MSB   0x0 // "Padc_MSB" (10-bit)
#define MPL115A2_REG_PRESSURE_LSB   0x1 // "Padc_LSB" (10-bit)
#define MPL115A2_REG_TEMP_MSB       0x2 // "Tadc_MSB" (10-bit)
#define MPL115A2_REG_TEMP_LSB       0x3 // "Tadc_LSB" (10-bit)
#define MPL115A2_REG_A0_COEFF_MSB   0x4 // "a0_MSB" (16-bit)
#define MPL115A2_REG_A0_COEFF_LSB   0x5 // "a0_LSB" (16-bit)
#define MPL115A2_REG_B1_COEFF_MSB   0x6 // "b1_MSB" (16-bit)
#define MPL115A2_REG_B1_COEFF_LSB   0x7 // "b1_LSB" (16-bit)
#define MPL115A2_REG_B2_COEFF_MSB   0x8 // "b2_MSB" (16-bit)
#define MPL115A2_REG_B2_COEFF_LSB   0x9 // "b2_LSB" (16-bit)
#define MPL115A2_REG_C12_COEFF_MSB  0xA // "c12_MSB" (14-bit)
#define MPL115A2_REG_C12_COEFF_LSB  0xB // "c12_LSB" (14-bit)
#define MPL115A2_REG_CONVERT        0x12 // "CONVERT"


#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}

static esp_err_t read_calibration_data(mpl115a2_t *dev)
{
    // read a0_msb, a0_lsb, b1_msb, b1_lsb, b2_msb, b2_lsb, c12_msb, c12_lsb
    uint8_t reg = MPL115A2_REG_A0_COEFF_MSB;
    uint8_t buf[8] = { 0 };
    
    CHECK(i2c_dev_read(&dev->i2c_dev, &reg, 1, buf, 8));

    /*
    ESP_LOGW(TAG, "buf[0]=%x", buf[0]);
    ESP_LOGW(TAG, "buf[1]=%x", buf[1]);
    ESP_LOGW(TAG, "buf[2]=%x", buf[2]);
    ESP_LOGW(TAG, "buf[3]=%x", buf[3]);
    ESP_LOGW(TAG, "buf[4]=%X", buf[4]);
    ESP_LOGW(TAG, "buf[5]=%X", buf[5]);
    ESP_LOGW(TAG, "buf[6]=%X", buf[6]);
    ESP_LOGW(TAG, "buf[7]=%X", buf[7]);
    */
    
    dev->coeff_a0 = (((uint16_t)buf[0] << 8) | buf[1]);
    dev->coeff_b1 = (((uint16_t)buf[2] << 8) | buf[3]);
    dev->coeff_b2 = (((uint16_t)buf[4] << 8) | buf[5]);
    dev->coeff_c12 = (((uint16_t)buf[6] << 8) | buf[7]) >> 2;

    dev->c_a0 = (float) dev->coeff_a0;
    dev->c_b1 = (float) dev->coeff_b1;
    dev->c_b2 = (float) dev->coeff_b2;
    dev->c_c12 = (float) dev->coeff_c12;
    
    if(dev->coeff_a0 & 0x8000)
        dev->c_a0 -= 65536;
    if(dev->coeff_b1 & 0x8000)
        dev->c_b1 -= 65536;
    if(dev->coeff_b2 & 0x8000)
        dev->c_b2 -= 65536;

    dev->c_a0 /= 8;
    dev->c_b1 /= 8192;
    dev->c_b2 /= 16384;
    dev->c_c12 /= 4194304.0;

    /*
    ESP_LOGW(TAG, "Calibration data received:");
    ESP_LOGW(TAG, "coeff_a0 bits=%X", dev->coeff_a0);
    ESP_LOGW(TAG, "coeff_b1 bits=%X", dev->coeff_b1);
    ESP_LOGW(TAG, "coeff_b2 bits=%X", dev->coeff_b2);
    ESP_LOGW(TAG, "coeff_c12 bits=%X", dev->coeff_c12);

    ESP_LOGW(TAG, "coeff_a0=%f", dev->c_a0);
    ESP_LOGW(TAG, "coeff_b1=%f", dev->c_b1);
    ESP_LOGW(TAG, "coeff_b2=%f", dev->c_b2);
    ESP_LOGW(TAG, "coeff_c12=%f", dev->c_c12);
    */
    
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MPL115A2_REG_CONVERT, 0), "Failed to start ADC conversion");
    
    return ESP_OK;
}

static esp_err_t take_reading(mpl115a2_t *dev, float *temperature,
                            float *pressure)
{

    float pressure_compensated;
    uint16_t padc = 0;
    uint16_t tadc = 0;

    uint8_t buf[4] = {0};
    uint8_t reg = MPL115A2_REG_PRESSURE_MSB;

    CHECK(i2c_dev_read(&dev->i2c_dev, &reg, 1, buf, 4));
    
    /*
    ESP_LOGW(TAG, "buf[0]=%X", buf[0]);
    ESP_LOGW(TAG, "buf[1]=%X", buf[1]);
    ESP_LOGW(TAG, "buf[2]=%X", buf[2]);
    ESP_LOGW(TAG, "buf[3]=%X", buf[3]);
    */
    
    padc = (((uint16_t)buf[0] << 8) | buf[1]) >> 6;
    tadc = (((uint16_t)buf[2] << 8) | buf[3]) >> 6;

    //ESP_LOGW(TAG, "padc=%X", padc);
    //ESP_LOGW(TAG, "tadc=%X", tadc);

    // See datasheet p.6 for evaluation sequence
    pressure_compensated = dev->c_a0 +
                 (dev->c_b1 + dev->c_c12 * tadc) * padc +
                 dev->c_b2 * tadc;

    //ESP_LOGD(TAG, "pcomp=%f", pressure_compensated);

    // Return pressure and temperature as floating point values
    // temperature formula from ladyada
    *pressure = (((65.0f / 1023.0f) * pressure_compensated) + 50.0f) * 1000; // kPa
    *temperature = ((float)tadc - 498.0f) / -5.35f + 25.f;    // C    

    return ESP_OK;
}

esp_err_t mpl115a2_init_desc(mpl115a2_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != MPL115A2_I2C_ADDRESS_0)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t mpl115a2_free_desc(mpl115a2_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mpl115a2_init(mpl115a2_t *dev)
{
    CHECK_ARG(dev);
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    CHECK_LOGE(dev, read_calibration_data(dev), "Failed to read calibration data");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mpl115a2_convert(mpl115a2_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MPL115A2_REG_CONVERT, 0), "Failed to start ADC conversion");
 
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mpl115a2_read_float(mpl115a2_t *dev, float *temperature,
                            float *pressure)
{
    CHECK_ARG(dev);
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MPL115A2_REG_CONVERT, 0), "Failed to start ADC conversion");
    
    // data sheet lists maximum time to do ADC conversion as 3 ms,
    // however esp32 vTaskDelay minimum is 1 tick (by default 10 ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    take_reading(dev, temperature, pressure);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mpl115a2_read_current_float(mpl115a2_t *dev, float *temperature,
                            float *pressure)
{
    CHECK_ARG(dev);
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    
    take_reading(dev, temperature, pressure);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}
