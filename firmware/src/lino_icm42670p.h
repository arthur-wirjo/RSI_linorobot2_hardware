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

#pragma message "COMPILING WITH ICM42670P"

#ifndef LINO_ICM42670P_H
#define LINO_ICM42670P_H

#include "imu_interface.h"
#include "ICM42670P.h" 
#include <Wire.h>
#include "MadgwickAHRS.h"

class ICM42670P_IMU : public IMUInterface
{
public:
    ICM42670P_IMU() : icm_sensor_(Wire, false) {}

    bool startSensor() override;
    geometry_msgs__msg__Vector3 readAccelerometer() override;
    geometry_msgs__msg__Vector3 readGyroscope() override;

    // ======================= INTERFACE FIX =======================
    // The function now takes a pointer and returns void.
    // This prevents a large, dangerous copy operation on the stack.
    void getIMUData(sensor_msgs__msg__Imu * imu_msg);
    // =============================================================
    
private:
    ICM42670 icm_sensor_;
    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;
    inv_imu_sensor_event_t sensor_event_;
    const float g_to_accel_ = 9.80665;
    Madgwick filter;
};

#endif // LINO_ICM42670P_H