/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *          https://www.bosch-sensortec.com/bst/products/all_products/bno055
 *          Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
 *
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <cerrno>
#include <chrono>
#include <thread>

#include "smbus_func.h"

#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/MotorData.hpp"
#include "interfaces/msg/fusionImu.hpp"
#include "interfaces/msg/rawImu.hpp"

/** BNO055 Address Alternative **/
#define BNO055_ADDRESS_A (0x28) // This requires the ADR pin on the bno055 to be low
/** BNO055 Address Default **/
#define BNO055_ADDRESS_DEFAULT (0x29) // This requires the ADR pin to the bno055 to be high
/** BNO055 Adress being used **/
#define BNO055_ADDRESS BNO055_ADDRESS_DEFAULT
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)
#define NUM_BNO055_QUATERNION_REGISTERS (8)
#define NUM_BNO055_EULER_REGISTERS (6)

namespace bno055_imu
{

    /** BNO055 Registers Adress **/
    typedef enum
    {
        /* Page id register definition */
        BNO055_PAGE_ID_ADDR = 0X07,

        /* PAGE0 REGISTERS */

        /* Chip IDs */
        BNO055_CHIP_ID_ADDR = 0x00,
        BNO055_ACCEL_REV_ID_ADDR = 0x01,
        BNO055_MAG_REV_ID_ADDR = 0x02,
        BNO055_GYRO_REV_ID_ADDR = 0x03,
        BNO055_SW_REV_ID_LSB_ADDR = 0x04,
        BNO055_SW_REV_ID_MSB_ADDR = 0x05,
        BNO055_BL_REV_ID_ADDR = 0X06,

        /* Acceleration data register */
        BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
        BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
        BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
        BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
        BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
        BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

        /* Magnetometer data register */
        BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
        BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
        BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
        BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
        BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
        BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

        /* Gyroscope data registers */
        BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
        BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
        BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
        BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
        BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
        BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

        /* Euler angels data registers */
        BNO055_EULER_H_LSB_ADDR = 0X1A,
        BNO055_EULER_H_MSB_ADDR = 0X1B,
        BNO055_EULER_R_LSB_ADDR = 0X1C,
        BNO055_EULER_R_MSB_ADDR = 0X1D,
        BNO055_EULER_P_LSB_ADDR = 0X1E,
        BNO055_EULER_P_MSB_ADDR = 0X1F,

        /* Quaternion data registers */
        BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
        BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
        BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
        BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
        BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
        BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
        BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
        BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

        /* Linear acceleration data registers */
        BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
        BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
        BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
        BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
        BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
        BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

        /* Gravity data registers */
        BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
        BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
        BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
        BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
        BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
        BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

        /* Temperature data register */
        BNO055_TEMP_ADDR = 0X34,

        /* Status registers */
        BNO055_CALIB_STAT_ADDR = 0X35,
        BNO055_SELFTEST_RESULT_ADDR = 0X36,
        BNO055_INTR_STAT_ADDR = 0X37,

        /* System Statues registers */
        BNO055_SYS_CLK_STAT_ADDR = 0X38,
        BNO055_SYS_STAT_ADDR = 0X39,
        BNO055_SYS_ERR_ADDR = 0X3A,

        /* Unit selection register */
        BNO055_UNIT_SEL_ADDR = 0X3B,

        /* Mode registers */
        BNO055_OPR_MODE_ADDR = 0X3D,
        BNO055_PWR_MODE_ADDR = 0X3E,

        BNO055_SYS_TRIGGER_ADDR = 0X3F,
        BNO055_TEMP_SOURCE_ADDR = 0X40,

        /* Axis remap registers */
        BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
        BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

        /* SIC registers */
        BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
        BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
        BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
        BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
        BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
        BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
        BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
        BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
        BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
        BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
        BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
        BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
        BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
        BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
        BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
        BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
        BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
        BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

        /* Accelerometer Offset registers */
        ACCEL_OFFSET_X_LSB_ADDR = 0X55,
        ACCEL_OFFSET_X_MSB_ADDR = 0X56,
        ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
        ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
        ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
        ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

        /* Magnetometer Offset registers */
        MAG_OFFSET_X_LSB_ADDR = 0X5B,
        MAG_OFFSET_X_MSB_ADDR = 0X5C,
        MAG_OFFSET_Y_LSB_ADDR = 0X5D,
        MAG_OFFSET_Y_MSB_ADDR = 0X5E,
        MAG_OFFSET_Z_LSB_ADDR = 0X5F,
        MAG_OFFSET_Z_MSB_ADDR = 0X60,

        /* Gyroscope Offset register s*/
        GYRO_OFFSET_X_LSB_ADDR = 0X61,
        GYRO_OFFSET_X_MSB_ADDR = 0X62,
        GYRO_OFFSET_Y_LSB_ADDR = 0X63,
        GYRO_OFFSET_Y_MSB_ADDR = 0X64,
        GYRO_OFFSET_Z_LSB_ADDR = 0X65,
        GYRO_OFFSET_Z_MSB_ADDR = 0X66,

        /* Radius registers */
        ACCEL_RADIUS_LSB_ADDR = 0X67,
        ACCEL_RADIUS_MSB_ADDR = 0X68,
        MAG_RADIUS_LSB_ADDR = 0X69,
        MAG_RADIUS_MSB_ADDR = 0X6A
    } reg_t;

    /** BNO055 power settings */
    typedef enum
    {
        POWER_MODE_NORMAL = 0X00,
        POWER_MODE_LOWPOWER = 0X01,
        POWER_MODE_SUSPEND = 0X02
    } powermode_t;

    /** Operation mode settings **/
    typedef enum
    {
        OPERATION_MODE_CONFIG = 0X00,
        OPERATION_MODE_ACCONLY = 0X01,
        OPERATION_MODE_MAGONLY = 0X02,
        OPERATION_MODE_GYRONLY = 0X03,
        OPERATION_MODE_ACCMAG = 0X04,
        OPERATION_MODE_ACCGYRO = 0X05,
        OPERATION_MODE_MAGGYRO = 0X06,
        OPERATION_MODE_AMG = 0X07,
        OPERATION_MODE_IMUPLUS = 0X08,
        OPERATION_MODE_COMPASS = 0X09,
        OPERATION_MODE_M4G = 0X0A,
        OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
        OPERATION_MODE_NDOF = 0X0C
    } opmode_t;

    /** Remap settings **/
    typedef enum
    {
        REMAP_CONFIG_P0 = 0x21,
        REMAP_CONFIG_P1 = 0x24, // default
        REMAP_CONFIG_P2 = 0x24,
        REMAP_CONFIG_P3 = 0x21,
        REMAP_CONFIG_P4 = 0x24,
        REMAP_CONFIG_P5 = 0x21,
        REMAP_CONFIG_P6 = 0x21,
        REMAP_CONFIG_P7 = 0x24
    } axis_remap_config_t;

    /** Remap Signs **/
    typedef enum
    {
        REMAP_SIGN_P0 = 0x04,
        REMAP_SIGN_P1 = 0x00, // default
        REMAP_SIGN_P2 = 0x06,
        REMAP_SIGN_P3 = 0x02,
        REMAP_SIGN_P4 = 0x03,
        REMAP_SIGN_P5 = 0x01,
        REMAP_SIGN_P6 = 0x07,
        REMAP_SIGN_P7 = 0x05
    } axis_remap_sign_t;

    typedef enum
    {
        UNITS_MS2 = 0x00, /**< Meters per second squared */
        UNITS_MG = 0x01,  /**< Mass times gravity */
    } accel_unit_t;

    typedef enum
    {
        UNITS_DPS = 0x00, /**< Degrees per second */
        UNITS_RPS = 0x02, /**< Radians per second */
    } angular_rate_unit_t;

    typedef enum
    {
        UNITS_DEGREES = 0x00, /**< Degrees */
        UNITS_RADIANS = 0x04, /**< Rdians */
    } euler_unit_t;

    typedef enum
    {
        UNITS_CELSIUS = 0x00,    /**< Degrees Celsius */
        UNITS_FAHRENHEIT = 0x10, /**< Degrees Fahrenheit */
    } temp_unit_t;

    /** A structure to configure the units **/
    typedef struct
    {
        accel_unit_t accel;
        angular_rate_unit_t angular_rate;
        euler_unit_t euler_angel;
        temp_unit_t temperature;
    } units_config_t;

    /** A structure to represent calibration offsets **/
    typedef struct
    {
        int16_t accel_offset_x; /**< x acceleration offset */
        int16_t accel_offset_y; /**< y acceleration offset */
        int16_t accel_offset_z; /**< z acceleration offset */
        int16_t mag_offset_x;   /**< x magnetometer offset */
        int16_t mag_offset_y;   /**< y magnetometer offset */
        int16_t mag_offset_z;   /**< z magnetometer offset */
        int16_t gyro_offset_x;  /**< x gyroscrope offset */
        int16_t gyro_offset_y;  /**< y gyroscrope offset */
        int16_t gyro_offset_z;  /**< z gyroscrope offset */
        int16_t accel_radius;   /**< acceleration radius */
        int16_t mag_radius;     /**< magnetometer radius */
    } offsets_t;

    /* A structur to represent imu measurements */
    typedef struct
    {
        int16_t acceleration_x;
        int16_t acceleration_y;
        int16_t acceleration_z;
        int16_t magnetometer_x;
        int16_t magnetometer_y;
        int16_t magnetometer_z;
        int16_t angular_velocity_x;
        int16_t angular_velocity_y;
        int16_t angular_velocity_z;
    } imu_data_raw_t;

    /* A structur to represent imu measurements */
    typedef struct
    {
        int16_t angular_velocity_x;
        int16_t angular_velocity_y;
        int16_t angular_velocity_z;
        int16_t euler_heading;
        int16_t euler_roll;
        int16_t euler_pitch;
        int16_t quaternion_w;
        int16_t quaternion_x;
        int16_t quaternion_y;
        int16_t quaternion_z;
        int16_t linear_acceleration_x;
        int16_t linear_acceleration_y;
        int16_t linear_acceleration_z;
        int16_t gravity_vector_x;
        int16_t gravity_vector_y;
        int16_t gravity_vector_z;
    } imu_data_t;

    class BNO055Driver
    {
    public:
        BNO055Driver(std::string device_, int address_, opmode_t opmode_);
        void init();
        void reset();
        // bool calibrate();
        bool read_imu_data(interfaces::msg::FusionImu &imu);
        bool read_imu_data_raw(interfaces::msg::RawImu &imu);
        int8_t get_temp(void);
        bool calibrate_sensor(void);

    private:
        int file;
        std::string _device;
        int _address;
        opmode_t _opmode;
        offsets_t _sensor_prof;

        void set_opmode(opmode_t);
        void write8(reg_t reg, __u8 value);
        __s32 read8(reg_t reg);
        void set_external_crystal(void);

        bool set_sensor_offsets(offsets_t);
        offsets_t get_sensor_offsets(void);
        bool save_sensor_profile(offsets_t);
        bool load_sensor_profile(void);
        bool isFullyCalibrated(void);
        void get_calibration_state(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
    };

}