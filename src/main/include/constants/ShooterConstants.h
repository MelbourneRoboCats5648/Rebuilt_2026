#pragma once
#include "units/acceleration.h"
#include <units/angle.h> 
#include <units/length.h>
#include <units/velocity.h>

using namespace units::acceleration;

namespace ShooterConstants {

inline constexpr meters_per_second_squared_t kAccelGravity = 9.81_mps_sq;
inline constexpr units::meter_t kXDist = 2_m;
inline constexpr units::meter_t kYDist = 1.888_m;
inline constexpr units::meter_t kIntialHeight = 0.3_m;
inline constexpr units::meters_per_second_t kShooterVelocity = 6.5_mps;

}

