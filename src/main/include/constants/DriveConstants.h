#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/current.h>
#include <units/time.h>
#include <units/length.h>

using namespace units::angular_velocity;
using namespace units::angular_acceleration;
using namespace units::current;
using namespace units::time;
using namespace units::length;

namespace DriveModuleConstants {
    namespace DirectionMotor {
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
        radians_per_second_t kMaxVel = 0.0_rad_per_s;
        radians_per_second_squared_t kMaxAcc = 0.0_rad_per_s_sq;

        ampere_t kMaxCurrent = 40_A;
        ampere_t kLowerCurrentLimit = 30_A;
        second_t kLowerLimitTime = 0.1_s;
    };
    namespace SpeedMotor {
        double kGearRatio = 1;
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;

        ampere_t kMaxCurrent = 50_A;
        ampere_t kLowerCurrentLimit = 60_A;
        second_t kLowerLimitTime = 0.1_s;

        meter_t kWheelCircumference = 0.5_m;
    };

};