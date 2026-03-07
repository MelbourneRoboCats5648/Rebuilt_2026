#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>

using namespace units::velocity;
using namespace units::acceleration;
using namespace units::angular_velocity;
using namespace units::angular_acceleration;
using namespace units::length;
using namespace units::voltage;

namespace IntakeConstants {
    inline constexpr double PI = 3.145; // fixme - get the const value from the correct math library

    inline constexpr meters_per_second_t kTargetIntakeSurfaceSpeed = 2.0_mps;  // desired linear surface speed of the intake wheel (fixme - check this value)

    inline constexpr meter_t kIntakeWheelDiameter = 0.1_m;  // fixme - check this value

    namespace intake { //fixme - tune PID constants
        inline constexpr double kP = 0.3;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        
        inline constexpr units::volt_t kS = 0.0_V;
        inline constexpr auto kV = 0.0_V / 1.0_tps;
        inline constexpr auto kA = 0.0_V / 1.0_tr_per_s_sq;
        
        inline constexpr units::turns_per_second_t kTolerance = 0.1_tps; // TODO
    }

    namespace extendRetract { //fixme - tune PID constants
        inline constexpr double kP = 3.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        
        inline constexpr units::volt_t kS = 0.0_V;
        inline constexpr units::volt_t kG = 0.0_V;
        inline constexpr auto kV = 0.0_V / 1.0_mps;
        inline constexpr auto kA = 0.0_V / 1.0_mps_sq;
        
        inline constexpr units::meters_per_second_t kMaxVelocity = 0.5_mps; // TODO
        inline constexpr units::meters_per_second_squared_t kMaxAcceleration = 1.0_mps_sq; // TODO
        
        inline constexpr units::meter_t kPositionTolerance = 0.01_m;
        inline constexpr units::meters_per_second_t kVelocityTolerance = 0.1_mps;
    }

    inline constexpr int kCurrentLimit(50);

    // Assume initial encoder position is 0 when fully retracted
    inline constexpr meter_t kExtendSoftLimit = 0.3_m;
    inline constexpr meter_t kRetractSoftLimit = 0.0_m;

    inline constexpr double kExtendRetractGearRatio(1.0 / 20.0); // 20 to 1 reduction gear
    inline constexpr double kExtendRetractSprocketDia(0.0254);   // 25.4 mm pitch diameter

    inline constexpr volt_t kMaxVoltage = 8_V;
};
