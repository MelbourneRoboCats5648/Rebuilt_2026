#pragma once
#include "units/acceleration.h"
#include <units/angle.h> 
#include <units/length.h>
#include <units/velocity.h>

using namespace units::acceleration;

namespace ShooterConstants {

inline constexpr meters_per_second_squared_t accelGravity = 9.81_mps_sq;
inline constexpr units::meter_t xDist = 2_m;
inline constexpr units::meter_t finalHeight = 1.888_m;
inline constexpr units::meters_per_second_t v = 6.5_mps;

}

