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

#include "bno055_imu_pub/bno055_driver.hpp"

namespace bno055_imu
{
    BNO055Driver::BNO055Driver(std::string device_, int address_, opmode_t opmode_)
    {
        _device = device_;
        _address = address_;
        _opmode = opmode_; // OPERATION_MODE_IMUPLUS;
    }

    void BNO055Driver::init()
    {

        /** open i2c device **/
        file = open(_device.c_str(), O_RDWR);
        if (file < 0)
        {
            throw std::runtime_error("Could not open i2c file");
        }

        /** Set the address for the target device (slave) **/
        if (ioctl(file, I2C_SLAVE, _address) < 0)
        {
            throw std::runtime_error("Could not open i2c device!!");
        }

        /** Verify we have the correct device **/
        if (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
        {
            throw std::runtime_error("incorrect chip ID");
        }

        // Reset the device
        reset();

        // set the external clock
        set_external_crystal();

        // Set the device to fusion imu mode
        set_opmode(_opmode);

        ROS_INFO_STREAM("BNO055 imu sensor was succesfully initiated.");
    }

    /**
     * @brief   Resets the bno055 sensor and waits until communication is
     *          resummed.
     */
    void BNO055Driver::reset(void)
    {
        uint8_t bno055_id = read8(BNO055_CHIP_ID_ADDR);
        set_opmode(OPERATION_MODE_CONFIG);
        ROS_INFO_STREAM("reseting chip");
        write8(BNO055_SYS_TRIGGER_ADDR, 0x20);

        /** Wait for the chip to restart **/
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        ROS_INFO_STREAM("after 800 millisecods.");
        if (bno055_id != read8(BNO055_CHIP_ID_ADDR))
        {
            throw std::runtime_error("Could not reset chip");
        }
    }

    /**
     *  @brief reads imu sensor measurments
     */
    bool BNO055Driver::read_imu_data(interfaces::msg::FusionImu &imu)
    {
        imu_data_t data;
        if (bno_i2c_smbus_read_i2c_block_data(file, BNO055_GYRO_DATA_X_LSB_ADDR, 32, (uint8_t *)&data) != 0x20)
        {
            throw std::runtime_error("read error");
            return false;
        }

        /* Gyroscope */
        /* 1dps = 16 LSB */
        /* 1rps = 900 LSB */
        imu.angular_velocity.x = ((double)data.angular_velocity_x) / 16.0;
        imu.angular_velocity.y = ((double)data.angular_velocity_y) / 16.0;
        imu.angular_velocity.z = ((double)data.angular_velocity_z) / 16.0;

        /* Euler Angles */
        /* 1 degree = 16 LSB */
        /* 1 radian = 900 LSB */
        imu.euler_angles.x = ((double)data.euler_roll) / 16.0;
        imu.euler_angles.y = ((double)data.euler_pitch) / 16.0;
        imu.euler_angles.z = ((double)data.euler_heading) / 16.0;

        /*!
         * Assign to Quaternion
         * See
         * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
         * 3.6.5.5 Orientation (Quaternion)
         */
        const double scale = (1.0 / (1 << 14));
        imu.orientation.w = (double)(data.quaternion_w * scale);
        imu.orientation.x = (double)(data.quaternion_x * scale);
        imu.orientation.y = (double)(data.quaternion_y * scale);
        imu.orientation.z = (double)(data.quaternion_z * scale);

        /* Linear acceleration */
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        imu.linear_acceleration.x = ((double)data.linear_acceleration_x) / 100.0;
        imu.linear_acceleration.y = ((double)data.linear_acceleration_y) / 100.0;
        imu.linear_acceleration.z = ((double)data.linear_acceleration_z) / 100.0;

        /* Gravity vector */
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        imu.gravity_vector.x = ((double)data.gravity_vector_x) / 100.0;
        imu.gravity_vector.y = ((double)data.gravity_vector_y) / 100.0;
        imu.gravity_vector.z = ((double)data.gravity_vector_z) / 100.0;

        return true;
    }

    /**
     *  @brief reads imu sensor measurments
     */
    bool BNO055Driver::read_imu_data_raw(interfaces::msg::RawImu &imu)
    {
        imu_data_raw_t data;
        if (bno_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 18, (uint8_t *)&data) != 18)
        {
            throw std::runtime_error("read error");
            return false;
        }

        /* Accelerometer */
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        imu.acceleration.x = ((double)data.acceleration_x) / 100.0;
        imu.acceleration.y = ((double)data.acceleration_y) / 100.0;
        imu.acceleration.z = ((double)data.acceleration_z) / 100.0;

        /* Magnetometer */
        /* 1uT = 16 LSB */
        imu.magnetometer.x = ((double)data.magnetometer_x) / 16.0;
        imu.magnetometer.y = ((double)data.magnetometer_y) / 16.0;
        imu.magnetometer.z = ((double)data.magnetometer_z) / 16.0;

        /* Gyroscope */
        /* 1dps = 16 LSB */
        /* 1rps = 900 LSB */
        imu.angular_velocity.x = ((double)data.angular_velocity_x) / 16.0;
        imu.angular_velocity.y = ((double)data.angular_velocity_y) / 16.0;
        imu.angular_velocity.z = ((double)data.angular_velocity_z) / 16.0;

        return true;
    }

    /**
     * @brief   Sets bno055 to one of the following operation mode:
     *              OPERATION_MODE_CONFIG
     *              OPERATION_MODE_ACCONLY
     *              OPERATION_MODE_MAGONLY
     *              OPERATION_MODE_GYRONLY
     *              OPERATION_MODE_ACCMAG
     *              OPERATION_MODE_ACCGYRO
     *              OPERATION_MODE_MAGGYRO
     *              OPERATION_MODE_AMG
     *              OPERATION_MODE_IMUPLUS        << This is the mode we use here>>
     *              OPERATION_MODE_COMPASS
     *              OPERATION_MODE_M4G
     *              OPERATION_MODE_NDOF_FMC_OFF
     *              OPERATION_MODE_NDOF
     *
     * @param op_mode   mask config of type opmode_t
     */
    void BNO055Driver::set_opmode(opmode_t opmode)
    {
        if (bno_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, opmode) < 0)
        {
            throw std::runtime_error("wirte error in opmode function");
        }
    }

    /**
     * @brief  Sets to use the external crystal (32.768KHz).
     */
    void BNO055Driver::set_external_crystal(void)
    {
        /* Switch to config mode */
        set_opmode(OPERATION_MODE_CONFIG);
        /* set correct page id */
        write8(BNO055_PAGE_ID_ADDR, 0);
        /* Set the external clock*/
        write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
        /* Set previouse operation mode */
        set_opmode(_opmode);
    }

    /**
     * @brief   Writes one byte of data to the BNO055 given register
     * @param   reg: Register address to write the data to.
     * @param   value: Data to be written in the register.
     */
    void BNO055Driver::write8(reg_t reg_, __u8 value)
    {
        if (bno_i2c_smbus_write_byte_data(file, reg_, value) < 0)
        {
            throw std::runtime_error("wirte error in write8 function");
        }
    }

    /**
     * @brief Reads one byte from the BNO055 (i2c slave) register
     * @param reg   Register address to read from.
     * @return Returns one byte of data read from the given register.
     */
    __s32 BNO055Driver::read8(reg_t reg_)
    {
        __s32 ret = bno_i2c_smbus_read_byte_data(file, reg_);
        if (ret < 0)
        {
            throw std::runtime_error("read error in read8 function");
        }
        return ret;
    }

    /**
     *  @brief  Gets the temperature in degrees celsius.
     *  @return temperature in degrees celsius.
     */
    int8_t BNO055Driver::get_temp(void)
    {
        int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
        return temp;
    }

    /**
     *  @brief  Writes to the sensor's offset registers.
     */
    bool BNO055Driver::set_sensor_offsets(offsets_t sensor_profile)
    {
        set_opmode(OPERATION_MODE_CONFIG);
        if (bno_i2c_smbus_write_i2c_block_data(file, ACCEL_OFFSET_X_LSB_ADDR, 0x17, (uint8_t *)&sensor_profile) != 0x17)
        {
            throw std::runtime_error("read error");
            return false;
        }
        set_opmode(_opmode);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if (!isFullyCalibrated())
        {
            return false;
        }
        return true;
    }

    /**
     *  @brief  Reads from the sensor's offset registers.
     *  @return returns a struct of type offset_t.
     */
    offsets_t BNO055Driver::get_sensor_offsets(void)
    {
        set_opmode(OPERATION_MODE_CONFIG);
        offsets_t calib_data;
        memset(&calib_data, 0, 0x17);
        if (bno_i2c_smbus_read_i2c_block_data(file, ACCEL_OFFSET_X_LSB_ADDR, 0x17, (uint8_t *)&calib_data) != 0x17)
        {
            throw std::runtime_error("read error");
        }
        return calib_data;
    }

    /**
     *  @brief  Checks that the operation mode is fully calibrated. The calibration state of a sensor is 3.
     *  @return TRUE - system is fully calibrated.
     *          FALSE - systme is not fully calibrated.
     */
    bool BNO055Driver::isFullyCalibrated(void)
    {
        uint8_t system, gyro, accel, mag;
        get_calibration_state(&system, &gyro, &accel, &mag);

        switch (read8(BNO055_OPR_MODE_ADDR))
        {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
        }
    }

    /**
     *  @brief  Gets current calibration state.  Each value should be a uint8_t
     *          pointer and it will be set to 0 if not calibrated and 3 if
     *          fully calibrated. See section 34.3.54 of bno055 datasheet.
     *
     * @param sys   systme calibration state.
     * @param gyro  gyroscope calibration state.
     * @param accel accelerometer calibration state.
     * @param mag   magnetometer calibration state.
     */
    void BNO055Driver::get_calibration_state(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
    {
        uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
        if (sys != NULL)
        {
            *sys = (calData >> 6) & 0x03;
        }
        if (gyro != NULL)
        {
            *gyro = (calData >> 4) & 0x03;
        }
        if (accel != NULL)
        {
            *accel = (calData >> 2) & 0x03;
        }
        if (mag != NULL)
        {
            *mag = calData & 0x03;
        }
    }

}