#pragma once

#include <units/length.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>

struct PIDConstants {
    double kP, kI, kD;
};

using namespace units::length;
using namespace units::current;

namespace ClimbConstants {
    inline constexpr meter_t kInitHeight = (67_m); 
    inline constexpr meter_t kDefaultHeight = (67_m);

    inline constexpr meter_t kExtendSoftLimit = (67_m);
    inline constexpr meter_t kRetractSoftLimit = (67_m);

    inline constexpr int kCurrentLimit = (50);

    inline constexpr double kGearRatio = (67);
      
    inline constexpr PIDConstants kClimbPID = {
    /* kP */ 0.2,
    /* kI */ 0.0,
    /* kD */ 0.0
  };

    inline constexpr units::second_t kDt = 20_ms;

    inline constexpr units::meter_t kElevatorPositionTolerance = 0.03_m;
    inline constexpr units::meters_per_second_t kElevatorVelocityTolerance = 0.1_mps;

}



