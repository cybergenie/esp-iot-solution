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
#include "ad7998.h"

static const char *TAG = "AD7998";

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */

uint8_t ad7998_ch_wakeup[16]={0x00,0x18,0x00,0x28,0x00,0x48,0x00,0x88,0x01,0x08,0x02,0x08,0x04,0x08,0x08,0x08};
uint8_t ad7998_ch_sleep[2]={0x00,0x08};

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between twice measurement, dt should be small (ms level) */
    struct timeval *timer;
} ad7998_dev_t;

ad7998_handle_t ad7998_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    if (bus == NULL) {
        return NULL;
    }

    ad7998_dev_t *sens = (ad7998_dev_t *) calloc(1, sizeof(ad7998_dev_t));
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    sens->counter = 0;
    sens->dt = 0;
    sens->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (ad7998_handle_t) sens;
}

esp_err_t ad7998_delete(ad7998_handle_t *sensor)
{
    if (*sensor == NULL) {
        return ESP_OK;
    }

    ad7998_dev_t *sens = (ad7998_dev_t *)(*sensor);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens->timer);
    free(sens);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t ad7998_get_deviceid(ad7998_handle_t sensor, uint8_t *deviceid)
{    
    return ESP_FAIL;
}

esp_err_t ad7998_wake_up(ad7998_handle_t sensor)
{ 
    ad7998_dev_t *sens = (ad7998_dev_t *) sensor;
    esp_err_t ret = i2c_bus_write_bytes(sens->i2c_dev, AD7998_CONFIG_ADDR, 2, ad7998_ch_wakeup);
    ESP_LOGI(TAG, "ad7998 actived\n");
    return ret;
}

esp_err_t ad7998_sleep(ad7998_handle_t sensor)
{    
    ad7998_dev_t *sens = (ad7998_dev_t *) sensor;
    esp_err_t ret = i2c_bus_write_bytes(sens->i2c_dev, AD7998_CONFIG_ADDR, 2, ad7998_ch_sleep);
    ESP_LOGI(TAG, "ad7998 is deactived\n");
    return ret;
}

esp_err_t ad7998_get_raw_data(ad7998_handle_t sensor, uint8_t *sensor_raw_data)
{
    ad7998_dev_t *sens = (ad7998_dev_t *) sensor;  
    esp_err_t ret=ESP_OK;  
    uint8_t data_rd[20] = {0xEE,0xEE};
    for(int i=1;i<=8;i++)
    {
        vTaskDelay(portTICK_PERIOD_MS);
        esp_err_t ret_2 = i2c_bus_write_bytes(sens->i2c_dev, AD7998_CONFIG_ADDR, 2, ad7998_ch_wakeup+(i-1)*2);
        vTaskDelay(portTICK_PERIOD_MS);
        if(ESP_OK==ret_2)
        {
            esp_err_t ret_1 = i2c_bus_read_bytes(sens->i2c_dev, AD7998_RESULT_ADDR, 2, data_rd+i*2);  
            ret=ret|ret_1;
        }      
         
    }   
    memcpy(sensor_raw_data,data_rd,sizeof(uint8_t)*20);
    return ret;
}


/***sensors hal interface****/
#ifdef CONFIG_SENSOR_GA_INCLUDED_AD7998

static ad7998_handle_t ad7998 = NULL;
static bool is_init = false;

esp_err_t ga_ad7998_init(i2c_bus_handle_t i2c_bus)
{
    if (is_init || !i2c_bus) {
        return ESP_FAIL;
    }

    ad7998 = ad7998_create(i2c_bus, AD7998_I2C_ADDRESS);

    if (!ad7998) {
        return ESP_FAIL;
    }

    uint8_t ad7998_deviceid = 0x00;
    ad7998_get_deviceid(ad7998, &ad7998_deviceid);
    ESP_LOGI(TAG, "ad7998 device address is: 0x%02x\n", ad7998_deviceid);
    esp_err_t ret = ad7998_wake_up(ad7998);

    if (ret == ESP_OK) {
        is_init = true;
    }

    return ret;
}

esp_err_t ga_ad7998_deinit(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    esp_err_t ret = ad7998_sleep(ad7998);
    ret = ad7998_delete(&ad7998);

    if (ret == ESP_OK) {
        is_init = false;
    }

    return ret;
}

esp_err_t ga_ad7998_sleep(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return ad7998_sleep(ad7998);
}

esp_err_t ga_ad7998_wakeup(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return ad7998_wake_up(ad7998);
}

esp_err_t ga_ad7998_test(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ga_ad7998_get_raw_data(uint8_t *ga_raw_data)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    uint8_t sensor_raw_data[20]={0};

    if(ga_raw_data !=NULL)
    {
        if(ESP_OK == ad7998_get_raw_data(ad7998,sensor_raw_data))
        {            
            memcpy(ga_raw_data,sensor_raw_data,sizeof(uint8_t)*20);
            return ESP_OK;
        }
    }
    memcpy(ga_raw_data,sensor_raw_data,sizeof(uint8_t)*20);
 


    return ESP_FAIL;
}



#endif