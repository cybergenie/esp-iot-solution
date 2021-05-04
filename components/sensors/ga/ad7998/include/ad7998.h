// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef __AD7998_H_
#define __AD7998_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c_bus.h"

#define AD7998_I2C_ADDRESS         0x21    /*!< slave address for AD7998 sensor */

/* AD7998 register */
#define AD7998_RESULT_ADDR         0x00 
#define AD7998_CONFIG_ADDR         0x02

// #ifdef CONFIG_AD7998_CH1
// uint8_t ad7998_ch_wakeup[2]={0x00,0x18};
// #endif
// #ifdef CONFIG_AD7998_CH2
// uint8_t ad7998_ch_wakeup[2]={0x00,0x28};
// #endif
// #ifdef CONFIG_AD7998_CH3
// uint8_t ad7998_ch_wakeup[2]={0x00,0x48};
// #endif
// #ifdef CONFIG_AD7998_CH4
// uint8_t ad7998_ch_wakeup[2]={0x00,0x88};
// #endif
// #ifdef CONFIG_AD7998_CH5
// uint8_t ad7998_ch_wakeup[2]={0x01,0x08};
// #endif
// #ifdef CONFIG_AD7998_CH6
// uint8_t ad7998_ch_wakeup[2]={0x02,0x08};
// #endif
// #ifdef CONFIG_AD7998_CH7
// uint8_t ad7998_ch_wakeup[2]={0x04,0x08};
// #endif
// #ifdef CONFIG_AD7998_CH8
// uint8_t ad7998_ch_wakeup[2]={0x08,0x08};
// #endif
// #ifdef CONFIG_AD7998_CH1_CH4
// uint8_t ad7998_ch_wakeup[2]={0x00,0xF8};
// #endif
// #ifdef CONFIG_AD7998_CH5_CH8
// uint8_t ad7998_ch_wakeup[2]={0x0F,0x08};
// #endif
// #ifdef CONFIG_AD7998_CH1_CH8
// uint8_t ad7998_ch_wakeup[2]={0x0F,0xF8};
// #endif



typedef void *ad7998_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
ad7998_handle_t ad7998_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of ad7998
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ad7998_delete(ad7998_handle_t *sensor);

/**
 * @brief Get device identification of AD7998
 *
 * @param sensor object handle of ad7998
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ad7998_get_deviceid(ad7998_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Wake up AD7998
 *
 * @param sensor object handle of ad7998
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ad7998_wake_up(ad7998_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of ad7998
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ad7998_sleep(ad7998_handle_t sensor);
/**
 * @brief Read raw sensor data 
 * 
 * @param sensor object handle of ad7998
 * 
 *  @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ad7998_get_raw_data(ad7998_handle_t sensor,uint8_t *sensor_raw_data);




/***implements of ga hal interface****/
#ifdef CONFIG_SENSOR_GA_INCLUDED_AD7998

/**
 * @brief initialize ad7998 with default configurations
 * 
 * @param i2c_bus i2c bus handle the sensor will attached to
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_init(i2c_bus_handle_t i2c_bus);

/**
 * @brief de-initialize ad7998
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_deinit(void);

/**
 * @brief test if ad7998 is active
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_test(void);

/**
 * @brief acquire ad7998 raw result one time.
 * 
 * @param raw result data

 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_get_raw_data(uint8_t *raw_data);





/**
 * @brief set ad7998 to sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_sleep(void);

/**
 * @brief wakeup ad7998 from sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_ad7998_wakeup(void);

#endif

#ifdef __cplusplus
}
#endif

#endif

