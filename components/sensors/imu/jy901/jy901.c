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
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"
#include "jy901.h"

static const char *TAG = "JY901";

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between twice measurement, dt should be small (ms level) */
    struct timeval *timer;
} jy901_dev_t;

jy901_handle_t jy901_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    if (bus == NULL) {
        return NULL;
    }

    jy901_dev_t *sens = (jy901_dev_t *) calloc(1, sizeof(jy901_dev_t));
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    sens->counter = 0;
    sens->dt = 0;
    sens->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (jy901_handle_t) sens;
}

esp_err_t jy901_delete(jy901_handle_t *sensor)
{
    if (*sensor == NULL) {
        return ESP_OK;
    }

    jy901_dev_t *sens = (jy901_dev_t *)(*sensor);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens->timer);
    free(sens);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t jy901_get_deviceid(jy901_handle_t sensor, uint8_t *deviceid)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, JY901_WHO_AM_I, &tmp);
    *deviceid = tmp;
    return ret;
}

esp_err_t jy901_wake_up(jy901_handle_t sensor)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, JY901_PWR_MGMT_1, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp &= (~BIT6);
    ret = i2c_bus_write_byte(sens->i2c_dev, JY901_PWR_MGMT_1, tmp);
    return ret;
}

esp_err_t jy901_sleep(jy901_handle_t sensor)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, JY901_PWR_MGMT_1, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp |= BIT6;
    ret = i2c_bus_write_byte(sens->i2c_dev, JY901_PWR_MGMT_1, tmp);
    return ret;
}

esp_err_t jy901_get_raw_data(jy901_handle_t sensor, uint8_t *sensor_raw_data)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;    
    uint8_t data_rd[20] = {0};
    esp_err_t ret_1 = i2c_bus_read_bytes(sens->i2c_dev, JY901_ACCEL_XOUT_H, 12, data_rd);
    esp_err_t ret_2 = i2c_bus_read_bytes(sens->i2c_dev, JY901_ROLL_OUT_H, 6, data_rd+12);
    memcpy(sensor_raw_data,data_rd,sizeof(uint8_t)*20);
    return ret_1 | ret_2;
}

esp_err_t jy901_get_raw_acce(jy901_handle_t sensor, jy901_raw_acce_value_t *raw_acce_value)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(sens->i2c_dev, JY901_ACCEL_XOUT_H, 6, data_rd);
    raw_acce_value->raw_acce_x = (int16_t)((data_rd[1] << 8)|(data_rd[0]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[3] << 8)|(data_rd[2]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[5] << 8)|(data_rd[4]));    
    return ret;
}

esp_err_t jy901_get_raw_gyro(jy901_handle_t sensor, jy901_raw_gyro_value_t *raw_gyro_value)
{
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(sens->i2c_dev, JY901_GYRO_XOUT_H, 6, data_rd);
    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[1] << 8)|(data_rd[0]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[3] << 8)|(data_rd[2]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[5] << 8)|(data_rd[4]));
    return ret;
}

esp_err_t jy901_get_acce(jy901_handle_t sensor, jy901_acce_value_t *acce_value)
{
    esp_err_t ret;    
    jy901_raw_acce_value_t raw_acce;   

    ret = jy901_get_raw_acce(sensor, &raw_acce);

    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = (raw_acce.raw_acce_x / 32768.0)*16*9.8;
    acce_value->acce_y = (raw_acce.raw_acce_y / 32768.0)*16*9.8;
    acce_value->acce_z = (raw_acce.raw_acce_z / 32768.0)*16*9.8;
    return ESP_OK;
}

esp_err_t jy901_get_gyro(jy901_handle_t sensor, jy901_gyro_value_t *gyro_value)
{
    esp_err_t ret;    
    jy901_raw_gyro_value_t raw_gyro; 

    ret = jy901_get_raw_gyro(sensor, &raw_gyro);

    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = (raw_gyro.raw_gyro_x / 32768.0)*2000.0;
    gyro_value->gyro_y = (raw_gyro.raw_gyro_y / 32768.0)*2000.0;
    gyro_value->gyro_z = (raw_gyro.raw_gyro_z / 32768.0)*2000.0;
    return ESP_OK;
}

esp_err_t jy901_complimentory_filter(jy901_handle_t sensor, jy901_acce_value_t *acce_value,
        jy901_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    jy901_dev_t *sens = (jy901_dev_t *) sensor;
    sens->counter++;

    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);
    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;
    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);
    return ESP_OK;
}

/***sensors hal interface****/
#ifdef CONFIG_SENSOR_IMU_INCLUDED_JY901

static jy901_handle_t jy901 = NULL;
static bool is_init = false;

esp_err_t imu_jy901_init(i2c_bus_handle_t i2c_bus)
{
    if (is_init || !i2c_bus) {
        return ESP_FAIL;
    }

    jy901 = jy901_create(i2c_bus, JY901_I2C_ADDRESS);

    if (!jy901) {
        return ESP_FAIL;
    }

    uint8_t jy901_deviceid;
    jy901_get_deviceid(jy901, &jy901_deviceid);
    ESP_LOGI(TAG, "jy901 device address is: 0x%02x\n", jy901_deviceid);
    esp_err_t ret = jy901_wake_up(jy901);

    if (ret == ESP_OK) {
        is_init = true;
    }

    return ret;
}

esp_err_t imu_jy901_deinit(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    esp_err_t ret = jy901_sleep(jy901);
    ret = jy901_delete(&jy901);

    if (ret == ESP_OK) {
        is_init = false;
    }

    return ret;
}

esp_err_t imu_jy901_sleep(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return jy901_sleep(jy901);
}

esp_err_t imu_jy901_wakeup(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return jy901_wake_up(jy901);
}

esp_err_t imu_jy901_test(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t imu_jy901_get_raw_data(uint8_t *imu_raw_data)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    uint8_t sensor_raw_data[20]={0};

    if(imu_raw_data !=NULL)
    {
        if(ESP_OK == jy901_get_raw_data(jy901,sensor_raw_data))
        {            
            memcpy(imu_raw_data,sensor_raw_data,sizeof(uint8_t)*20);
            return ESP_OK;
        }
    }
    memcpy(imu_raw_data,sensor_raw_data,sizeof(uint8_t)*20);
 


    return ESP_FAIL;
}

esp_err_t imu_jy901_acquire_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    jy901_gyro_value_t gyro = {0, 0, 0};

    if (gyro_x != NULL && gyro_y != NULL && gyro_z != NULL) {
        if (ESP_OK == jy901_get_gyro(jy901, &gyro)) {
            *gyro_x = gyro.gyro_x;
            *gyro_y = gyro.gyro_y;
            *gyro_z = gyro.gyro_z;
            return ESP_OK;
        }
    }

    *gyro_x = 0;
    *gyro_y = 0;
    *gyro_z = 0;
    return ESP_FAIL;
}

esp_err_t imu_jy901_acquire_acce(float *acce_x, float *acce_y, float *acce_z)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    jy901_acce_value_t acce = {0, 0, 0};

    if (acce_x != NULL && acce_y != NULL && acce_z != NULL) {
        if (ESP_OK == jy901_get_acce(jy901, &acce)) {
            *acce_x = acce.acce_x;
            *acce_y = acce.acce_y;
            *acce_z = acce.acce_z;
            return ESP_OK;
        }
    }

    *acce_x = 0;
    *acce_y = 0;
    *acce_z = 0;
    return ESP_FAIL;
}

#endif