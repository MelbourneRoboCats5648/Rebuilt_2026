#pragma once
#include <units/voltage.h>

namespace FeederConstants {

    inline constexpr units::volt_t kFeederVoltage(10_V);
    inline constexpr units::volt_t kSideFeederVoltage(5_V); // fixme

    inline constexpr int kCurrentLimit(50);

    inline constexpr units::volt_t kSideFeederVoltageDifferenceIncrement(0.1_V);
};