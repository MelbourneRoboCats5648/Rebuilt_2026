#pragma once

#include "units/length.h"
#include "units/angle.h"
#include <units/voltage.h>
#include "FieldConstants.h"


using namespace units::length;
using namespace units::angle;

namespace ShooterConstants {
    namespace motor {
        inline constexpr double kP = 0.0; // fixme - we'll need kP (1.0 previously)
        inline constexpr double kI = 0;
        inline constexpr double kD = 0;
        inline constexpr double kV = 0.4;
        inline constexpr double kS = 0.07;
        inline constexpr double kA = 0;
    }

    inline constexpr meter_t startHeight(0.3);
    inline constexpr meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;
    inline constexpr double kgearRatio = 2;

    inline constexpr units::volt_t kMaxVoltage(12);
    inline constexpr units::turns_per_second_t kMaxAngularVelocity(50);
    inline constexpr inch_t kFlyWheelRadius(2_in);
};