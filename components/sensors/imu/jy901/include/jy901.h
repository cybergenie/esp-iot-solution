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
#define JY901_SELF_TEST_X         0x0D
#define JY901_SELF_TEST_Y         0x0E
#define JY901_SELF_TEST_Z         0x0F
#define JY901_SELF_TEST_A         0x10
#define JY901_SMPLRT_DIV          0x19
#define JY901_CONFIG              0x1A
#define JY901_GYRO_CONFIG         0x1B
#define JY901_ACCEL_CONFIG        0x1C
#define JY901_FIFO_EN             0x23
#define JY901_I2C_MST_CTRL        0x24
#define JY901_I2C_SLV0_ADDR       0x25
#define JY901_I2C_SLV0_REG        0x26
#define JY901_I2C_SLV0_CTRL       0x27
#define JY901_I2C_SLV1_ADDR       0x28
#define JY901_I2C_SLV1_REG        0x29
#define JY901_I2C_SLV1_CTRL       0x2A
#define JY901_I2C_SLV2_ADDR       0x2B
#define JY901_I2C_SLV2_REG        0x2C
#define JY901_I2C_SLV2_CTRL       0x2D
#define JY901_I2C_SLV3_ADDR       0x2E
#define JY901_I2C_SLV3_REG        0x2F
#define JY901_I2C_SLV3_CTRL       0x30
#define JY901_I2C_SLV4_ADDR       0x31
#define JY901_I2C_SLV4_REG        0x32
#define JY901_I2C_SLV4_DO         0x33
#define JY901_I2C_SLV4_CTRL       0x34
#define JY901_I2C_SLV4_DI         0x35
#define JY901_I2C_MST_STATUS      0x36
#define JY901_INT_PIN_CFG         0x37
#define JY901_INT_ENABLE          0x38
#define JY901_DMP_INT_STATUS      0x39
#define JY901_INT_STATUS          0x3A
#define JY901_ACCEL_XOUT_H        0x34
#define JY901_ACCEL_XOUT_L        0x34
#define JY901_ACCEL_YOUT_H        0x35
#define JY901_ACCEL_YOUT_L        0x35
#define JY901_ACCEL_ZOUT_H        0x36
#define JY901_ACCEL_ZOUT_L        0x36
#define JY901_TEMP_OUT_H          0x40
#define JY901_TEMP_OUT_L          0x40
#define JY901_GYRO_XOUT_H         0x37
#define JY901_GYRO_XOUT_L         0x37
#define JY901_GYRO_YOUT_H         0x38
#define JY901_GYRO_YOUT_L         0x38
#define JY901_GYRO_ZOUT_H         0x39
#define JY901_GYRO_ZOUT_L         0x39
#define JY901_EXT_SENS_DATA_00    0x49
#define JY901_EXT_SENS_DATA_01    0x4A
#define JY901_EXT_SENS_DATA_02    0x4B
#define JY901_EXT_SENS_DATA_03    0x4C
#define JY901_EXT_SENS_DATA_04    0x4D
#define JY901_EXT_SENS_DATA_05    0x4E
#define JY901_EXT_SENS_DATA_06    0x4F
#define JY901_EXT_SENS_DATA_07    0x50
#define JY901_EXT_SENS_DATA_08    0x51
#define JY901_EXT_SENS_DATA_09    0x52
#define JY901_EXT_SENS_DATA_10    0x53
#define JY901_EXT_SENS_DATA_11    0x54
#define JY901_EXT_SENS_DATA_12    0x55
#define JY901_EXT_SENS_DATA_13    0x56
#define JY901_EXT_SENS_DATA_14    0x57
#define JY901_EXT_SENS_DATA_15    0x58
#define JY901_EXT_SENS_DATA_16    0x59
#define JY901_EXT_SENS_DATA_17    0x5A
#define JY901_EXT_SENS_DATA_18    0x5B
#define JY901_EXT_SENS_DATA_19    0x5C
#define JY901_EXT_SENS_DATA_20    0x5D
#define JY901_EXT_SENS_DATA_21    0x5E
#define JY901_EXT_SENS_DATA_22    0x5F
#define JY901_EXT_SENS_DATA_23    0x60
#define JY901_I2C_SLV0_DO         0x63
#define JY901_I2C_SLV1_DO         0x64
#define JY901_I2C_SLV2_DO         0x65
#define JY901_I2C_SLV3_DO         0x66
#define JY901_I2C_MST_DELAY_CTRL  0x67
#define JY901_SIGNAL_PATH_RESET   0x68
#define JY901_USER_CTRL           0x6A
#define JY901_PWR_MGMT_1          0x6B
#define JY901_PWR_MGMT_2          0x6C
#define JY901_FIFO_COUNTH         0x72
#define JY901_FIFO_COUNTL         0x73
#define JY901_FIFO_R_W            0x74
#define JY901_WHO_AM_I            0x75

typedef enum {
    ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} jy901_acce_fs_t;

typedef enum {
    GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} jy901_gyro_fs_t;

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} jy901_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} jy901_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} jy901_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} jy901_gyro_value_t;

typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

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
esp_err_t jy901_get_raw_data(jy901_handle_t sensor);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of jy901
 * @param acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_raw_acce(jy901_handle_t sensor, jy901_raw_acce_value_t *raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of jy901
 * @param gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_raw_gyro(jy901_handle_t sensor, jy901_raw_gyro_value_t *raw_gyro_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of jy901
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_acce(jy901_handle_t sensor, jy901_acce_value_t *acce_value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of jy901
 * @param gyro_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_get_gyro(jy901_handle_t sensor, jy901_gyro_value_t *gyro_value);

/**
 * @brief use complimentory filter to caculate roll and pitch
 *
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t jy901_complimentory_filter(jy901_handle_t sensor, jy901_acce_value_t *acce_value,
        jy901_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle);

/***implements of imu hal interface****/
#ifdef CONFIG_SENSOR_IMU_INCLUDED_JY901

/**
 * @brief initialize jy901 with default configurations
 * 
 * @param i2c_bus i2c bus handle the sensor will attached to
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_init(i2c_bus_handle_t i2c_bus);

/**
 * @brief de-initialize jy901
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_deinit(void);

/**
 * @brief test if jy901 is active
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_test(void);

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
esp_err_t imu_jy901_get_raw_data(uint8_t *raw_data);

/**
 * @brief acquire jy901 accelerometer result one time.
 * 
 * @param acce_x result data (unit:g)
 * @param acce_y result data (unit:g)
 * @param acce_z result data (unit:g)
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_acquire_acce(float *acce_x, float *acce_y, float *acce_z);

/**
 * @brief acquire jy901 gyroscope result one time.
 * 
 * @param gyro_x result data (unit:dps)
 * @param gyro_y result data (unit:dps)
 * @param gyro_z result data (unit:dps)
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_acquire_gyro(float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief set jy901 to sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_sleep(void);

/**
 * @brief wakeup jy901 from sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_jy901_wakeup(void);

#endif

#ifdef __cplusplus
}
#endif

#endif

