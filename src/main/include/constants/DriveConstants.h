#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

using namespace units::angular_velocity;
using namespace units::angular_acceleration;

namespace DriveModuleConstants {
    namespace DirectionController {
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
        radians_per_second_t kMaxVel = 0.0_rad_per_s;
        radians_per_second_squared_t kMaxAcc = 0.0_rad_per_s_sq;
    };
};