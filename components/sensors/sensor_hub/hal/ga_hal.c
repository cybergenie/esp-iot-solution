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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/ga_hal.h"

#ifdef CONFIG_SENSOR_GA_INCLUDED_JY901
#include "jy901.h"
#endif

#ifdef CONFIG_SENSOR_GA_INCLUDED_AD7998
#include "ad7998.h"
#endif


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static esp_err_t null_function(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static esp_err_t null_acquire_function(float *x, float *y, float *z)
{
    return ESP_ERR_NOT_SUPPORTED;
}
#pragma GCC diagnostic pop

static const char *TAG = "GA";

#define SENSOR_CHECK(a, str, ret) if(!(a)) { \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret); \
    }

typedef struct {
    ga_id_t id;
    esp_err_t (*init)(bus_handle_t);
    esp_err_t (*deinit)(void);
    esp_err_t (*test)(void);
    esp_err_t (*acquire_raw_data)(uint8_t *raw_data);
    esp_err_t (*sleep)(void);
    esp_err_t (*wakeup)(void);
} ga_impl_t;

typedef struct {
    ga_id_t id;
    bus_handle_t bus;
    bool is_init;
    const ga_impl_t *impl;
} sensor_ga_t;

static const ga_impl_t ga_implementations[] = {
#ifdef CONFIG_SENSOR_GA_INCLUDED_JY901
    {
        .id = JY901_ID,
        .init = ga_jy901_init,
        .deinit = ga_jy901_deinit,
        .test = ga_jy901_test,        
        .acquire_raw_data = ga_jy901_get_raw_data,
        .sleep = ga_jy901_sleep,
        .wakeup = ga_jy901_wakeup,
    },
#endif
#ifdef CONFIG_SENSOR_GA_INCLUDED_AD7998
    {
        .id = AD7998_ID,
        .init = ga_ad7998_init,
        .deinit = ga_ad7998_deinit,
        .test = ga_ad7998_test,        
        .acquire_raw_data = ga_ad7998_get_raw_data,
        .sleep = ga_ad7998_sleep,
        .wakeup = ga_ad7998_wakeup,
    },
#endif
};

/****************************private functions*************************************/

static const ga_impl_t *find_implementation(ga_id_t id)
{
    const ga_impl_t *active_driver = NULL;
    int count = sizeof(ga_implementations) / sizeof(ga_impl_t);

    for (int i = 0; i < count; i++) {
        if (ga_implementations[i].id == id) {
            active_driver = &ga_implementations[i];
            break;
        }
    }

    return active_driver;
}

/****************************public functions*************************************/

sensor_ga_handle_t ga_create(bus_handle_t bus, int ga_id)
{
    SENSOR_CHECK(bus != NULL, "i2c bus has not initialized", NULL);
    const ga_impl_t *sensor_impl = find_implementation(ga_id);

    if (sensor_impl == NULL) {
        ESP_LOGE(TAG, "no driver founded, GA ID = %d", ga_id);
        return NULL;
    }

    sensor_ga_t *p_sensor = (sensor_ga_t *)malloc(sizeof(sensor_ga_t));
    SENSOR_CHECK(p_sensor != NULL, "ga sensor creat failed", NULL);
    p_sensor->id = ga_id;
    p_sensor->bus = bus;
    p_sensor->impl = sensor_impl;
    esp_err_t ret = p_sensor->impl->init(bus);

    if (ret != ESP_OK) {
        free(p_sensor);
        ESP_LOGE(TAG, "ga sensor init failed");
        return NULL;
    }

    p_sensor->is_init = true;
    return (sensor_ga_handle_t)p_sensor;
}

esp_err_t ga_delete(sensor_ga_handle_t *sensor)
{
    SENSOR_CHECK(sensor != NULL && *sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(*sensor);

    if (!p_sensor->is_init) {
        free(p_sensor);
        return ESP_OK;
    }

    p_sensor->is_init = false;
    esp_err_t ret = p_sensor->impl->deinit();
    SENSOR_CHECK(ret == ESP_OK, "ga sensor de-init failed", ESP_FAIL);
    free(p_sensor);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t ga_test(sensor_ga_handle_t sensor)
{
    SENSOR_CHECK(sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(sensor);

    if (!p_sensor->is_init) {
        return ESP_FAIL;
    }

    esp_err_t ret = p_sensor->impl->test();
    return ret;
}

esp_err_t ga_acquire_raw(sensor_ga_handle_t sensor, uint8_t* raw_data)
{
    SENSOR_CHECK(sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t* )(sensor);
    esp_err_t ret = p_sensor->impl->acquire_raw_data(raw_data);
    return ret;
}


esp_err_t ga_sleep(sensor_ga_handle_t sensor)
{
    SENSOR_CHECK(sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(sensor);
    esp_err_t ret = p_sensor->impl->sleep();
    return ret;
}

esp_err_t ga_wakeup(sensor_ga_handle_t sensor)
{
    SENSOR_CHECK(sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(sensor);
    esp_err_t ret = p_sensor->impl->wakeup();
    return ret;
}

static esp_err_t ga_set_power(sensor_ga_handle_t sensor, sensor_power_mode_t power_mode)
{
    SENSOR_CHECK(sensor != NULL, "pointer can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(sensor);
    esp_err_t ret;
    switch (power_mode)
    {
    case POWER_MODE_WAKEUP:
        ret = p_sensor->impl->wakeup();
        break;
    case POWER_MODE_SLEEP:
        ret = p_sensor->impl->sleep();
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    return ret;
}

esp_err_t ga_acquire(sensor_ga_handle_t sensor, sensor_data_group_t *data_group)
{
    SENSOR_CHECK(sensor != NULL && data_group != NULL, "pointer can't be NULL ", ESP_ERR_INVALID_ARG);
    sensor_ga_t *p_sensor = (sensor_ga_t *)(sensor);
    esp_err_t ret;
    int i = 0;

     ret = p_sensor->impl->acquire_raw_data(&data_group->sensor_data[i].sensor_raw_data);
    if (ESP_OK == ret) {
        data_group->sensor_data[i].event_id = SENSOR_RAW_DATA_READY;
        i++;
    }
    
    data_group->number = i;
    return ESP_OK;
}

esp_err_t ga_control(sensor_ga_handle_t sensor, sensor_command_t cmd, void *args)
{
    SENSOR_CHECK(sensor != NULL, "sensor handle can't be NULL ", ESP_ERR_INVALID_ARG);
    esp_err_t ret;
    switch (cmd)
    {
    case COMMAND_SET_MODE:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    case COMMAND_SET_RANGE:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    case COMMAND_SET_ODR:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    case COMMAND_SET_POWER:
        ret = ga_set_power(sensor, (sensor_power_mode_t)args);
        break;
    case COMMAND_SELF_TEST:
        ret = ga_test(sensor);
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    return ret;
}
