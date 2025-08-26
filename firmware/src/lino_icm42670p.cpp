// Copyright [2025] [Your Name]
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

#include <Arduino.h>
#include "lino_icm42670p.h"
#include "config.h"
#include <Wire.h>

#define SAMPLE_RATE_HZ 100

// This function is now stable with the required delays.
bool ICM42670P_IMU::startSensor()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100); 
    
    int status = icm_sensor_.begin();
    if (status < 0) {
        return false;
    }
    delay(100);

    uint16_t accel_odr = 1000; 
    uint16_t gyro_odr = 1000;
    uint16_t accel_fsr = 16;
    uint16_t gyro_fsr = 2000;

    status = icm_sensor_.startAccel(accel_odr, accel_fsr);
    if (status < 0) { return false; }

    status = icm_sensor_.startGyro(gyro_odr, gyro_fsr);
    if (status < 0) { return false; }

    filter.begin(SAMPLE_RATE_HZ);
    delay(100);
    return true;
}

// ======================= IMPLEMENTATION FIX =======================
// The function now operates directly on the provided pointer,
// eliminating the need for a local variable and a dangerous copy.
void ICM42670P_IMU::getIMUData(sensor_msgs__msg__Imu * imu_msg)
{
    static char frame_id[] = "imu_link";
    imu_msg->header.frame_id.data = frame_id;
    imu_msg->header.frame_id.size = strlen(frame_id);
    imu_msg->header.frame_id.capacity = strlen(frame_id) + 1;

    icm_sensor_.getDataFromRegisters(sensor_event_);

    const float ACCEL_RAW_TO_G = 1.0f / 2048.0f;
    const float GYRO_RAW_TO_DPS = 1.0f / 16.4f;

    float gyroX_dps = (float)sensor_event_.gyro[0] * GYRO_RAW_TO_DPS;
    float gyroY_dps = (float)sensor_event_.gyro[1] * GYRO_RAW_TO_DPS;
    float gyroZ_dps = (float)sensor_event_.gyro[2] * GYRO_RAW_TO_DPS;

    float accelX_g = (float)sensor_event_.accel[0] * ACCEL_RAW_TO_G;
    float accelY_g = (float)sensor_event_.accel[1] * ACCEL_RAW_TO_G;
    float accelZ_g = (float)sensor_event_.accel[2] * ACCEL_RAW_TO_G;

    if ((accelX_g != 0.0f) || (accelY_g != 0.0f) || (accelZ_g != 0.0f))
    {
        filter.updateIMU(gyroX_dps, gyroY_dps, gyroZ_dps, accelX_g, accelY_g, accelZ_g);
    }

    float roll_rad = filter.getRoll() * DEG_TO_RAD;
    float pitch_rad = filter.getPitch() * DEG_TO_RAD;
    float yaw_rad = filter.getYaw() * DEG_TO_RAD;
    
    float cy = cos(yaw_rad * 0.5f);
    float sy = sin(yaw_rad * 0.5f);
    float cp = cos(pitch_rad * 0.5f);
    float sp = sin(pitch_rad * 0.5f);
    float cr = cos(roll_rad * 0.5f);
    float sr = sin(roll_rad * 0.5f);

    imu_msg->orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg->orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg->orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg->orientation.z = cr * cp * sy - sr * sp * cy;
    
    imu_msg->angular_velocity.x = gyroX_dps * DEG_TO_RAD;
    imu_msg->angular_velocity.y = gyroY_dps * DEG_TO_RAD;
    imu_msg->angular_velocity.z = gyroZ_dps * DEG_TO_RAD;

    imu_msg->linear_acceleration.x = accelX_g * g_to_accel_;
    imu_msg->linear_acceleration.y = accelY_g * g_to_accel_;
    imu_msg->linear_acceleration.z = accelZ_g * g_to_accel_;

    imu_msg->orientation_covariance[0] = 0.01;
    imu_msg->orientation_covariance[4] = 0.01;
    imu_msg->orientation_covariance[8] = 0.01;
    imu_msg->angular_velocity_covariance[0] = 0.001;
    imu_msg->angular_velocity_covariance[4] = 0.001;
    imu_msg->angular_velocity_covariance[8] = 0.001;
    imu_msg->linear_acceleration_covariance[0] = 0.01;
    imu_msg->linear_acceleration_covariance[4] = 0.01;
    imu_msg->linear_acceleration_covariance[8] = 0.01;
}
// =================================================================

geometry_msgs__msg__Vector3 ICM42670P_IMU::readAccelerometer()
{
    icm_sensor_.getDataFromRegisters(sensor_event_);
    const float ACCEL_RAW_TO_G = 1.0f / 2048.0f;
    accel_.x = (float)sensor_event_.accel[0] * ACCEL_RAW_TO_G * g_to_accel_;
    accel_.y = (float)sensor_event_.accel[1] * ACCEL_RAW_TO_G * g_to_accel_;
    accel_.z = (float)sensor_event_.accel[2] * ACCEL_RAW_TO_G * g_to_accel_;
    return accel_;
}

geometry_msgs__msg__Vector3 ICM42670P_IMU::readGyroscope()
{
    icm_sensor_.getDataFromRegisters(sensor_event_);
    const float GYRO_RAW_TO_DPS = 1.0f / 16.4f;
    gyro_.x = ((float)sensor_event_.gyro[0] * GYRO_RAW_TO_DPS) * DEG_TO_RAD;
    gyro_.y = ((float)sensor_event_.gyro[1] * GYRO_RAW_TO_DPS) * DEG_TO_RAD;
    gyro_.z = ((float)sensor_event_.gyro[2] * GYRO_RAW_TO_DPS) * DEG_TO_RAD;
    return gyro_;
}