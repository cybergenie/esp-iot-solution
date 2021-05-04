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
#ifndef __JY901_H_
#define __JY901_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c_bus.h"

#define JY901_I2C_ADDRESS         0x50    /*!< slave address for JY901 sensor */

/* JY901 register */
#define JY901_ACCEL_XOUT_H        0x34
#define JY901_ACCEL_XOUT_L        0x34
#define JY901_ACCEL_YOUT_H        0x35
#define JY901_ACCEL_YOUT_L        0x35
#define JY901_ACCEL_ZOUT_H        0x36
#define JY901_ACCEL_ZOUT_L        0x36
#define JY901_GYRO_XOUT_H         0x37
#define JY901_GYRO_XOUT_L         0x37
#define JY901_GYRO_YOUT_H         0x38
#define JY901_GYRO_YOUT_L         0x38
#define JY901_GYRO_ZOUT_H         0x39
#define JY901_GYRO_ZOUT_L         0x39
#define JY901_ROLL_OUT_H          0x3d
#define JY901_ROLL_OUT_L          0x3d
#define JY901_PITCH_OUT_H         0x3e
#define JY901_PITCH_OUT_L         0x3e
#define JY901_YAW_OUT_H           0x3f
#define JY901_YAW_OUT_L           0x3f
#define JY901_TEMP_OUT_H          0x40
#define JY901_TEMP_OUT_L          0x40
#define JY901_PWR_MGMT_1          0x22
#define JY901_WHO_AM_I            0x22

typedef void *jy901_handle_t;

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
jy901_handle_t jy901_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of jy901
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_delete(jy901_handle_t *sensor);

/**
 * @brief Get device identification of JY901
 *
 * @param sensor object handle of jy901
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_deviceid(jy901_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Wake up JY901
 *
 * @param sensor object handle of jy901
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_wake_up(jy901_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of jy901
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_sleep(jy901_handle_t sensor);
/**
 * @brief Read raw sensor data 
 * 
 * @param sensor object handle of jy901
 * 
 *  @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_raw_data(jy901_handle_t sensor,uint8_t *sensor_raw_data);




/***implements of ga hal interface****/
#ifdef CONFIG_SENSOR_GA_INCLUDED_JY901

/**
 * @brief initialize jy901 with default configurations
 * 
 * @param i2c_bus i2c bus handle the sensor will attached to
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_init(i2c_bus_handle_t i2c_bus);

/**
 * @brief de-initialize jy901
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_deinit(void);

/**
 * @brief test if jy901 is active
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_test(void);

/**
 * @brief acquire jy901 raw result one time.
 * 
 * @param raw result data
 * 0x34 AX X 轴加速度   2bit
 * 0x35 AY Y 轴加速度   2bit
 * 0x36 AZ Z 轴加速度   2bit
 * 0x37 GX X 轴角速度   2bit
 * 0x38 GY Y 轴角速度   2bit
 * 0x39 GZ Z 轴角速度   2bit
 * 0x3d Roll X 轴角度   2bit
 * 0x3e Pitch Y 轴角度  2bit
 * 0x3f Yaw Z 轴角度    2bit
 * 0x40 TEMP 模块温度   2bit
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_get_raw_data(uint8_t *raw_data);





/**
 * @brief set jy901 to sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_sleep(void);

/**
 * @brief wakeup jy901 from sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_jy901_wakeup(void);

#endif

#ifdef __cplusplus
}
#endif

#endif

