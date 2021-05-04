// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
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

#ifndef _GA_HAL_H_
#define _GA_HAL_H_

#include "i2c_bus.h"
#include "esp_err.h"
#include "sensor_type.h"

typedef void *sensor_ga_handle_t; /*!< ga sensor handle*/

/**
 * @brief ga sensor id, used for ga_create
 * 
 */
typedef enum {
    JY901_ID = 0x01, /*!< MPU6050 ga sensor id*/    
    GA_MAX_ID, /*!< max ga sensor id*/
} ga_id_t;

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Create a Inertial Measurement Unit sensor instance.
 * Same series' sensor or sensor with same address can only be created once.
 *
 * @param bus i2c bus handle the sensor attached to
 * @param ga_id id declared in ga_id_t
 * @return sensor_ga_handle_t return ga sensor handle if succeed, NULL is failed.
 */
sensor_ga_handle_t ga_create(bus_handle_t bus, int ga_id);

/**
 * @brief Delete and release the sensor resource.
 *
 * @param sensor point to ga sensor handle, will set to NULL if delete succeed.
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
*/
esp_err_t ga_delete(sensor_ga_handle_t *sensor);

/**
 * @brief Test if sensor is active.
 *
 * @param sensor ga sensor handle to operate
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
*/
esp_err_t ga_test(sensor_ga_handle_t sensor);

/**
 * @brief Set sensor to sleep mode.
 *
 * @param sensor ga sensor handle to operate
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_NOT_SUPPORTED Function not supported on this sensor
*/
esp_err_t ga_sleep(sensor_ga_handle_t sensor);

/**
 * @brief Wakeup sensor from sleep mode.
 *
 * @param sensor ga sensor handle to operate
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_NOT_SUPPORTED Function not supported on this sensor
*/
esp_err_t ga_wakeup(sensor_ga_handle_t sensor);


esp_err_t ga_acquire_raw(sensor_ga_handle_t sensor, uint8_t* raw_data);

/**
 * @brief acquire a group of sensor data
 * 
 * @param sensor ga sensor handle to operate
 * @param data_group acquired data
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ga_acquire(sensor_ga_handle_t sensor, sensor_data_group_t *data_group);

/**
 * @brief control sensor mode with control commands and args
 * 
 * @param sensor ga sensor handle to operate
 * @param cmd control commands detailed in sensor_command_t
 * @param args control commands args
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_NOT_SUPPORTED Function not supported on this sensor
 */
esp_err_t ga_control(sensor_ga_handle_t sensor, sensor_command_t cmd, void *args);

#ifdef __cplusplus
extern "C"
}
#endif


#endif
