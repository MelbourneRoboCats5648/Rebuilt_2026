#pragma once

#include "units/length.h"
#include "units/angle.h"
#include <units/voltage.h>
#include "FieldConstants.h"


using namespace units::length;
using namespace units::angle;

namespace ShooterConstants {
    namespace motor {
        inline constexpr double kP = 0;
        inline constexpr double kI = 0;
        inline constexpr double kD = 0;
    }

    inline constexpr meter_t distance(2.0);
    inline constexpr degree_t angle(60);
    inline constexpr meter_t startHeight(0.3);
    inline constexpr meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;
    inline constexpr double kgearRatio = 1;

    inline constexpr units::volt_t kMaxVoltage(12);

};

