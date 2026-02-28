#pragma once

#include "units/length.h"
#include "units/angle.h"
#include "units/acceleration.h"

#include "frc/geometry/Translation2d.h"


using namespace units::length;
using namespace units::angle;
using namespace units::acceleration;

namespace FieldConstants {

    inline constexpr meter_t HubHeight(1.888);
    inline constexpr meters_per_second_squared_t gravity(9.8);

    inline constexpr frc::Translation2d kHubPosition {0_m, 0_m}; //fixme
}