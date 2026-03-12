#pragma once

#include "units/length.h"
#include "units/angle.h"
#include "units/acceleration.h"

#include "frc/geometry/Translation2d.h"


using namespace units::length;
using namespace units::angle;
using namespace units::acceleration;

namespace FieldConstants {

    inline constexpr meter_t HubHeight(1.83);
    inline constexpr meters_per_second_squared_t gravity(9.8);

    inline constexpr frc::Translation2d kHubPosition {182.11_in, 158.84_in};
    inline constexpr frc::Translation2d kPositionOffset {0_in, 50_in};  // fixme - target position offset in Y either side of hub

}