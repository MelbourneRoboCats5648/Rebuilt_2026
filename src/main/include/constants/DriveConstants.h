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
        inline constexpr double kP = 6.0; // equivalent to 0.9 V/(t/s)
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr radians_per_second_t kMaxVel = (2 * M_PI) * 2_rad_per_s;
        inline constexpr radians_per_second_squared_t kMaxAcc = (2 * M_PI) * 4_rad_per_s_sq;

        inline constexpr ampere_t kMaxCurrent = 40_A;
        inline constexpr ampere_t kLowerCurrentLimit = 30_A;
        inline constexpr second_t kLowerLimitTime = 0.1_s;
    };

    namespace SpeedMotor {
        inline constexpr double kGearRatio = 6.75;
        inline constexpr double kP = 2.0;    // value of 2.8 introduces instability on the 2025 robot base
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double kS = 0.0959800231072556; // found by linear regression (LR)
        inline constexpr double kV = 0.776168369501521; // found by LR in units of Volts per (turns per second)
        inline constexpr double kA = 0.0;    // tried tuning this but appears to be unused
        // fixme: tune on final robot

        inline constexpr ampere_t kMaxCurrent = 50_A;
        inline constexpr ampere_t kLowerCurrentLimit = 60_A;
        inline constexpr second_t kLowerLimitTime = 0.1_s;

        inline constexpr meter_t kWheelRadius = 0.0508_m;
        inline constexpr meter_t kWheelCircumference = 2 * std::numbers::pi * kWheelRadius;
    };

    namespace DirectionEncoder {
        // mag offset zero position requies all swerve module bolts pointing towards the right (negative Y axis of robot) 
        inline constexpr turn_t kFrontLeftOffset = 0.07568359375_tr;
        inline constexpr turn_t kFrontRightOffset = 0.325927734375_tr;
        inline constexpr turn_t kBackLeftOffset = -0.055419921875_tr;
        inline constexpr turn_t kBackRightOffset = 0.26953125_tr;
    }; // fixme - mag offsets are 180 deg off - make sure we are at the right direction before taking measurements
};

namespace DrivetrainConstants {
    namespace ModuleLocation {
        inline constexpr frc::Translation2d kFrontLeft{+0.2569_m, +0.2569_m};
        inline constexpr frc::Translation2d kFrontRight{+0.2569_m, -0.2569_m};
        inline constexpr frc::Translation2d kBackLeft{-0.2569_m, +0.2569_m};
        inline constexpr frc::Translation2d kBackRight{-0.2569_m, -0.2569_m};
    };

    inline constexpr meters_per_second_t kMaxSpeed = 5.0_mps; // fixme - will need to increase this for comp
    inline constexpr meters_per_second_squared_t kMaxAcceleration = 3.0_mps_sq;

    inline constexpr radians_per_second_t kMaxAngularSpeed = (2 * M_PI) * 1_rad_per_s;
    inline constexpr radians_per_second_squared_t kMaxAngularAcceleration = (2 * M_PI) * 1_rad_per_s_sq;

    inline constexpr degree_t kInitialGyroAngle = 0_deg;

    // with front facing away from DS
    inline constexpr degree_t kInitialBlueHeading = 0_deg;
    inline constexpr degree_t kInitialRedHeading = 180_deg;

    namespace Autonomous {
        namespace XYController {
            inline constexpr double kP = 3.0;
            inline constexpr double kI = 0.0;
            inline constexpr double kD = 0.0;
            inline constexpr meter_t kTolerance = 0.05_m;
        };
        namespace ThetaController {
            inline constexpr double kP = 3.0;
            inline constexpr double kI = 0.0;
            inline constexpr double kD = 0.0;
            inline constexpr radian_t kPositionTolerance = 1_deg;
            inline constexpr radians_per_second_t kVelocityTolerance = 1_deg_per_s;
        };

        inline constexpr double kTrajTimeTolerance = 0.1; // additional ratio on top of total traj time
    }; // NOTE: repurposed for Choreo controller
};