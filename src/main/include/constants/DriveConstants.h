#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/current.h>
#include <units/time.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#include <frc/geometry/Translation2d.h>

using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::angular_acceleration;
using namespace units::current;
using namespace units::time;
using namespace units::length;
using namespace units::velocity;
using namespace units::acceleration;

namespace DriveModuleConstants {
    namespace DirectionMotor {
        inline constexpr double kP = 6.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr radians_per_second_t kMaxVel = M_PI * 4_rad_per_s;
        inline constexpr radians_per_second_squared_t kMaxAcc = M_PI * 8_rad_per_s_sq;

        inline constexpr ampere_t kMaxCurrent = 40_A;
        inline constexpr ampere_t kLowerCurrentLimit = 30_A;
        inline constexpr second_t kLowerLimitTime = 0.1_s;
    };

    namespace SpeedMotor {
        inline constexpr double kGearRatio = 6.75;
        inline constexpr double kP = 0.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double kS = 0.1438; // found by linear regression (LR)
        inline constexpr double kV = 0.7797; // found by LR
        inline constexpr double kA = 0.0;    // tried tuning this but appears to be unused

        inline constexpr ampere_t kMaxCurrent = 50_A;
        inline constexpr ampere_t kLowerCurrentLimit = 60_A;
        inline constexpr second_t kLowerLimitTime = 0.1_s;

        inline constexpr meter_t kWheelRadius = 0.0508_m;
        inline constexpr meter_t kWheelCircumference = 2 * std::numbers::pi * kWheelRadius;
    };

    namespace DirectionEncoder {
        inline constexpr turn_t kFrontLeftOffset = -0.306640625_tr;
        inline constexpr turn_t kFrontRightOffset = -0.1748046875_tr;
        inline constexpr turn_t kBackLeftOffset = 0.01953125_tr;
        inline constexpr turn_t kBackRightOffset = 0.07421875_tr;
    };
};

namespace DrivetrainConstants {
    namespace ModuleLocation {
        inline constexpr frc::Translation2d kFrontLeft{+0.26_m, +0.26_m};
        inline constexpr frc::Translation2d kFrontRight{+0.26_m, -0.26_m};
        inline constexpr frc::Translation2d kBackLeft{-0.26_m, +0.26_m};
        inline constexpr frc::Translation2d kBackRight{-0.26_m, -0.26_m};
    };

    inline constexpr meters_per_second_t kMaxSpeed = 1.0_mps;
    inline constexpr meters_per_second_squared_t kMaxAcceleration = 1.0_mps_sq;

    inline constexpr radians_per_second_t kMaxAngularSpeed = 3.142_rad_per_s;
    inline constexpr radians_per_second_squared_t kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    inline constexpr degree_t kInitialGyroAngle = 0_deg;
};