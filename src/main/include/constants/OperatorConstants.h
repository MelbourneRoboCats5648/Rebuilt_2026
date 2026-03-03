// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// PID Profile and Controller stuff
#include <units/acceleration.h>
#include <units/angular_acceleration.h>


namespace OperatorConstants {
    inline constexpr int kDriverControllerPort = 0;
    inline constexpr int kMechControllerPort = 1;
    inline constexpr double kDeadband = 0.1; //Driver??
    inline constexpr double kMechDeadband = 0.1;

    inline constexpr units::meters_per_second_squared_t kSlewRateTranslation = 6_mps_sq; //increase to reduce lag
    inline constexpr units::radians_per_second_squared_t kSlewRateRotation = 6_rad_per_s_sq;


};
