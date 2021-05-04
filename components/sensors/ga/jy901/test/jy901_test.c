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
#include "unity.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "jy901.h"
#include "esp_system.h"

#define I2C_MASTER_SCL_IO           22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static jy901_handle_t jy901 = NULL;

/**
 * @brief i2c master initialization
 */
static void jy901_test_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    jy901 = jy901_create(i2c_bus, JY901_I2C_ADDRESS);
}

static void jy901_test_deinit()
{
    jy901_delete(&jy901);
    i2c_bus_delete(&i2c_bus);
}

static void jy901_test_get_data()
{
    uint8_t jy901_deviceid;
    jy901_acce_value_t acce;
    jy901_gyro_value_t gyro;
    int cnt = 10;
    jy901_get_deviceid(jy901, &jy901_deviceid);
    printf("jy901 device ID is: 0x%02x\n", jy901_deviceid);
    jy901_wake_up(jy901);
    jy901_set_acce_fs(jy901, ACCE_FS_4G);
    jy901_set_gyro_fs(jy901, GYRO_FS_500DPS);

    while (cnt--) {
        printf("\n************* JY901 MOTION SENSOR ************\n");
        jy901_get_acce(jy901, &acce);
        printf("acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
        jy901_get_gyro(jy901, &gyro);
        printf("gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        printf("**************************************************\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

TEST_CASE("Sensor jy901 test get data [1000ms]", "[jy901][iot][sensor]")
{
    jy901_test_init();
    vTaskDelay(1000 / portTICK_RATE_MS);
    jy901_test_get_data();
    jy901_test_deinit();
}
