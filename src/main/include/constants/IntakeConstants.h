#pragma once

#include <units/velocity.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

using namespace units::velocity;
using namespace units::angular_velocity;
using namespace units::length;

namespace IntakeConstants {
    inline constexpr double PI = 3.145; // fixme - get the const value from the correct math library

    inline constexpr meters_per_second_t kTargetIntakeSurfaceSpeed = 2.0_mps;  // desired linear surface speed of the intake wheel (fixme - check this value)

    inline constexpr meter_t kIntakeWheelDiameter = 0.1_m;  // fixme - check this value

    namespace intake { //fixme - tune PID constants
        inline constexpr double kP = 0.3;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
    }

    namespace extendRetract { //fixme - tune PID constants
        inline constexpr double kP = 0.3;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
    }


    inline constexpr int kCurrentLimit(50);
    inline constexpr meter_t kExtendSoftLimit = (67_m); //fixme - may not be necessary 
    inline constexpr meter_t kRetractSoftLimit = (67_m); //fixme - may not be necessary 

    inline constexpr int kIntakeGearRatio(5);
};
