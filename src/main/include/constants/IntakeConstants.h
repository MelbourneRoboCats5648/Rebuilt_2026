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
        inline constexpr double kP = 1.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
    }


    inline constexpr int kCurrentLimit(50);

    // Assume initial encoder position is 0 when fully retracted
    inline constexpr meter_t kExtendSoftLimit = (0.25_m); //fixme - tune this value (the slide has 300mm extension)
    inline constexpr meter_t kRetractSoftLimit = (0.0_m); //fixme - tune this

    inline constexpr double kExtendRetractGearRatio(1.0 / 20.0); // 20 to 1 reduction gear
    inline constexpr double kExtendRetractSprocketDia(0.0254); // 25.4 mm pitch diameter
};
