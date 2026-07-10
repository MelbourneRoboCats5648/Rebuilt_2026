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
    inline constexpr meters_per_second_t kTargetIntakeSurfaceSpeed = 8.0_mps;  // desired linear surface speed of the intake wheel
    // 7.5-8.0 m/s is doable if there's a net, otherwise use 7.0 m/s

    inline constexpr meter_t kIntakeWheelDiameter = 4.0_in;

    namespace intake {
        inline constexpr double kP = 0.1;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        
        inline constexpr units::volt_t kS = 0.1164_V;
        inline constexpr auto kV = 0.1104_V / 1.0_tps;
        inline constexpr auto kA = 0.0_V / 1.0_tr_per_s_sq;
        
        inline constexpr units::turns_per_second_t kTolerance = 0.1_tps;
    }

    namespace extendRetract {
        inline constexpr double kP = 0.0; // fixme(MRT) - only using feedforward. Might need to add kP if required
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        
        inline constexpr units::volt_t kS = 0.13109_V;
        inline constexpr units::volt_t kG = 0.02394_V;
        inline constexpr auto kV = 29.904_V / 1.0_mps; // avg of extend and retract slopes
        inline constexpr auto kA = 0.0_V / 1.0_mps_sq;
        
        inline constexpr units::meters_per_second_t kMaxExtendVelocity = 0.30_mps;
        inline constexpr units::meters_per_second_squared_t kMaxExtendAcceleration = 0.8_mps_sq;
    
        inline constexpr units::meters_per_second_t kMaxRetractVelocity = 0.15_mps;
        inline constexpr units::meters_per_second_squared_t kMaxRetractAcceleration = 0.4_mps_sq;
        
        inline constexpr units::meter_t kPositionTolerance = 0.01_m; // fixme(#55) - check position tolerance along with desired velocity
        inline constexpr units::meters_per_second_t kVelocityTolerance = 0.1_mps;

        inline constexpr units::meters_per_second_t kCalibrationVelocity = 0.01_mps; // for retracting to zero pos
    }

    inline constexpr int kCurrentLimit(50);

    // Assume initial encoder position is 0 when fully retracted
    inline constexpr meter_t kExtendSoftLimit = 0.23_m;
    inline constexpr meter_t kRetractSoftLimit = 0.0_m;

    inline constexpr double kExtendRetractGearRatio(1.0 / 20.0); // 20 to 1 reduction gear
    inline constexpr double kExtendRetractSprocketDia(0.0254);   // 25.4 mm pitch diameter
    inline constexpr double kExtendRetractPulleyRatio(46.0 / 44.0); // 46 to 44 pulley ratio (46 output, 44 input)
    

    inline constexpr volt_t kMaxVoltage = 2_V;
};
