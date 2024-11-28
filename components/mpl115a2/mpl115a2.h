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
 * @file mpl115a2.h
 * @defgroup mpl115a2 mpl115a2
 * @{
 *
 * ESP-IDF driver for mpl115a2 temperature and pressure sensor
 *
 * Adopted from esp-idf-lib bmp280 driver.
 *
 * Copyright (c) 2016 sheinz <https://github.com/sheinz>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2023 xhedit <https://github.com/xhedit>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MPL115A2_H__
#define __MPL115A2_H__

#include <stdint.h>
#include <stdbool.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPL115A2_I2C_ADDRESS_0  0x60 //!< I2C address when SDO pin is low



/**
 * Device descriptor
 */
typedef struct {
    uint16_t coeff_a0;
    uint8_t coeff_a0_MSB;
    uint8_t coeff_a0_LSB;
    uint16_t coeff_b1;
    uint8_t coeff_b1_MSB;
    uint8_t coeff_b1_LSB;
    uint16_t coeff_b2;
    uint8_t coeff_b2_MSB;
    uint8_t coeff_b2_LSB;
    uint16_t coeff_c12;
    uint8_t coeff_c12_MSB;
    uint8_t coeff_c12_LSB;

    float c_a0;
    float c_b1;
    float c_b2;
    float c_c12;

    i2c_dev_t i2c_dev;  //!< I2C device descriptor
} mpl115a2_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr mpl115a2 address
 * @param port I2C port number
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_init_desc(mpl115a2_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_free_desc(mpl115a2_t *dev);

/**
 * @brief Initialize MPL115A2 module
 *
 * Probes for the device, soft resets the device, reads the calibration
 * constants.
 *
 * This may be called again to soft reset the device and initialize it again.
 *
 * @param dev Device descriptor
 * @param params Parameters
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_init(mpl115a2_t *dev);

/**
 * @brief Start measurement
 *
 * Fill the registers with a new adc reading.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_convert(mpl115a2_t *dev);

/**
 * @brief Calculate coefficients
 *
 * Calculate coefficients from the register data.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_calc_coeff(mpl115a2_t *dev);

/**
 * @brief Read compensated temperature and pressure data
 *
 * Request a new reading and return it.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature (deg. C)
 * @param[out] pressure Pressure (kPa)
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_read_float(mpl115a2_t *dev, float *temperature,
                            float *pressure);

/**
 * @brief Read compensated temperature and pressure data
 *
 * Return the current saved compensated temp and pressure reading.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature (deg. C)
 * @param[out] pressure Pressure (kPa)
 * @return `ESP_OK` on success
 */
esp_err_t mpl115a2_read_current_float(mpl115a2_t *dev, float *temperature,
                            float *pressure);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP280_H__
