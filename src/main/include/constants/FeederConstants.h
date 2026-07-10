#pragma once
#include <units/voltage.h>

namespace FeederConstants {

    inline constexpr units::volt_t kFeederVoltage(10_V);
    inline constexpr units::volt_t kSideFeederVoltage(6_V); 

    inline constexpr int kCurrentLimit(50);
    inline constexpr double kSideMotorGearRatio(3.0);

    inline constexpr double kStallCurrentThreshold(15.0);
};