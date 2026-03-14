#pragma once

#include <units/length.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>

#include <frc/trajectory/TrapezoidProfile.h>

struct PIDConstants {
    double kP, kI, kD;
};

using namespace units::length;
using namespace units::current;
using namespace units::voltage;

namespace ClimbConstants {
    
    //motors

    //fixme - change and add parameters later. placeholder values atm. 

    // inline constexpr meter_t kInitHeight = (67_m); 
    // inline constexpr meter_t kDefaultHeight = (67_m);

    //inline constexpr meter_t kExtendSoftLimit = (67_m);  
    //inline constexpr meter_t kRetractSoftLimit = (0_m);

    inline constexpr int kCurrentLimit = (50);
    inline constexpr volt_t kMaxVoltage = 5_V; // fixme - increase for comp

    inline constexpr double kGearRatio = (1/60.0);

    inline constexpr double kClimbSprocketDia(0.0363);   // 1.432 inches in metres

    //PID 
    //inline constexpr frc::TrapezoidProfile<units::meter>::Constraints trapezoidProfileClimb{0.6_mps, 2.0_mps_sq};
      
    // inline constexpr PIDConstants kClimbPID = {
    // /* kP */ 0.2,
    // /* kI */ 0.0,
    // /* kD */ 0.0
    // };

    // inline constexpr units::second_t kDt = 20_ms;

    inline constexpr units::meter_t kClimbPositionTolerance = 0.03_m;
    inline constexpr units::meters_per_second_t kClimbVelocityTolerance = 0.1_mps;

}



