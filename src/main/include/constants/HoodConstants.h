#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/time.h>

namespace HoodConstants {
    inline constexpr units::meter_t hoodHeight(0.39);

    inline constexpr units::degree_t kMinAngleSoftLimit = 38.0470_deg;
    inline constexpr units::degree_t kMaxAngleSoftLimit = 68.1986_deg;

    inline constexpr units::degree_t kMinAngle = kMinAngleSoftLimit + 0.5_deg; // actual testing shows that the min angle achievable with PID is 55.13 deg due to stiction
    inline constexpr units::degree_t kMaxAngle = kMaxAngleSoftLimit - 0.5_deg; // max angle achievable with PID is very close to 67 deg
    inline constexpr units::degree_t kMidAngle = (kMinAngle + kMaxAngle) / 2;
    inline constexpr units::meter_t kRangeThreshold = 2.0_m;

    inline const units::degree_t kMaxAngleRange = kMaxAngleSoftLimit - kMinAngleSoftLimit;
    inline const double kAngleDegreesPerTurn = kMaxAngleRange.value() / (18.016512 - 5.683217);

    inline constexpr units::volt_t kCalibrationVoltage = 2.0_V;
    inline constexpr units::degrees_per_second_t kCalibrationVelocityThreshold = 1.5_deg_per_s;
    inline constexpr units::second_t kCalibrationPreTime = 0.05_s;
    inline constexpr units::second_t kCalibrationTimeout = 1.5_s;

    inline constexpr double kP = 0.15;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;

};