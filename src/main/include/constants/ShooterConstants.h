#pragma once

#include "units/length.h"
#include "units/angle.h"
#include "FieldConstants.h"
#include "units/acceleration.h"


using namespace units::length;
using namespace units::angle;
using namespace units::acceleration;

namespace ShooterConstants {

    inline constexpr meter_t distance (2.0);
    inline constexpr degree_t angle (60);
    inline constexpr meter_t startHeight(0.3);
    inline constexpr meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;
    inline constexpr meter_t XDist(2); 
    inline constexpr meter_t YDist(1.888); 
    inline constexpr meter_t IntialHeight(0.3);
    inline constexpr meters_per_second_t ShooterVelocity(6.5_mps);


};