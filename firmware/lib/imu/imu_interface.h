// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>
#include <cmath>

#ifndef ACCEL_COV
#define ACCEL_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef GYRO_COV
#define GYRO_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef ORI_COV
#define ORI_COV { 0.00001, 0.00001, 0.00001 }
#endif

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        const float accel_cov[3] = ACCEL_COV;
        const float gyro_cov[3] = GYRO_COV;
        const float ori_cov[3] = ORI_COV;
        const int sample_size_ = 40;

        geometry_msgs__msg__Vector3 gyro_cal_;

        // --- ADD THE FOLLOWING BLOCK OF CODE ---
        // State variables for the Complementary Filter
        float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
        unsigned long last_update_time_ = 0;
        // --- END OF ADDED BLOCK ---

        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }

    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
            // --- ADD THE FOLLOWING LINE ---
            // A covariance of -1 indicates that orientation data is not available yet.
            imu_msg_.orientation_covariance[0] = -1;
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }
/*
        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
#ifndef USE_MPU6050_IMU // mpu6050 already calibrated in driver
            imu_msg_.angular_velocity.x -= gyro_cal_.x;
            imu_msg_.angular_velocity.y -= gyro_cal_.y;
            imu_msg_.angular_velocity.z -= gyro_cal_.z;
#endif

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
            imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
            imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
            imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];

            imu_msg_.orientation_covariance[0] = ori_cov[0];
            imu_msg_.orientation_covariance[4] = ori_cov[1];
            imu_msg_.orientation_covariance[8] = ori_cov[2];

#ifdef IMU_TWEAK
            IMU_TWEAK
#endif
            return imu_msg_;
        }
*/

// DELETE THE OLD getData() METHOD AND REPLACE IT WITH THIS ONE

sensor_msgs__msg__Imu getData()
{
    // --- Start of Sensor Fusion Logic ---

    // Ensure the timer is initialized on the first run
    if (last_update_time_ == 0) {
        last_update_time_ = micros();
    }

    // 1. Calculate the time difference (delta time) since the last update
    unsigned long current_time = micros();
    float dt = (current_time - last_update_time_) / 1000000.0F;
    last_update_time_ = current_time;

    // 2. Read raw data from the sensors
    imu_msg_.angular_velocity = readGyroscope();
    imu_msg_.linear_acceleration = readAccelerometer();

    // 3. Perform Gyroscope Calibration (this replaces the old calibrateGyro() logic)
#ifndef USE_MPU6050_IMU // mpu6050 is calibrated in its driver
    imu_msg_.angular_velocity.x -= gyro_cal_.x;
    imu_msg_.angular_velocity.y -= gyro_cal_.y;
    imu_msg_.angular_velocity.z -= gyro_cal_.z;
#endif

    // 4. Run the Complementary Filter to calculate orientation

    // Get Roll and Pitch from the Accelerometer (stable, long-term reference)
    float accel_roll = atan2(imu_msg_.linear_acceleration.y, imu_msg_.linear_acceleration.z);
    float accel_pitch = atan2(-imu_msg_.linear_acceleration.x, 
        sqrt(pow(imu_msg_.linear_acceleration.y, 2) + pow(imu_msg_.linear_acceleration.z, 2)));

    // Integrate Gyroscope data to get the change in angle (responsive, short-term)
    roll_ += imu_msg_.angular_velocity.x * dt;
    pitch_ += imu_msg_.angular_velocity.y * dt;
    yaw_ += imu_msg_.angular_velocity.z * dt; // Yaw will drift without a magnetometer

    // Fuse the two estimates. 98% from gyro, 2% from accel.
    const float alpha = 0.98;
    roll_ = alpha * roll_ + (1.0 - alpha) * accel_roll;
    pitch_ = alpha * pitch_ + (1.0 - alpha) * accel_pitch;

    // 5. Convert the final Euler angles (roll, pitch, yaw) to a Quaternion
    float cr = cos(roll_ * 0.5);
    float sr = sin(roll_ * 0.5);
    float cp = cos(pitch_ * 0.5);
    float sp = sin(pitch_ * 0.5);
    float cy = cos(yaw_ * 0.5);
    float sy = sin(yaw_ * 0.5);

    // This is the missing piece you identified! We are now populating the orientation.
    imu_msg_.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg_.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg_.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg_.orientation.z = cr * cp * sy - sr * sp * cy;

    // 6. Populate covariance matrices
    imu_msg_.orientation_covariance[0] = ori_cov[0];
    imu_msg_.orientation_covariance[4] = ori_cov[1];
    imu_msg_.orientation_covariance[8] = ori_cov[2];

    imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
    imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
    imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];

    imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
    imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
    imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];

#ifdef IMU_TWEAK
    IMU_TWEAK
#endif
    return imu_msg_;
}

};

#endif
