#pragma once

#include "units/length.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include <units/voltage.h>
#include "FieldConstants.h"
#include <units/current.h>

using namespace units::length;
using namespace units::angle;

namespace FlyWheelConstants {
    namespace motor {
        inline constexpr double kP = 0.2;
        inline constexpr double kI = 0;
        inline constexpr double kD = 0;
        inline constexpr double kV = 0.22956;
        inline constexpr double kS = 0.087569;
        inline constexpr double kA = 0;
    }

    // fixme(MRT) - rename startHeight to hood height. Check this is still 0.3 metres
    inline constexpr meter_t startHeight(0.3);
    // fixme(MRT) - below variable isn't being used. Could be deleted
    inline constexpr meter_t adjustedHeight = FieldConstants::HubHeight - startHeight;
    inline constexpr double kgearRatio = 1.0;

    inline constexpr units::meter_t kFlyWheelRadius(2_in);

    inline constexpr int kCurrentLimit(50);  // fixme(MRT) - check if max angular velocity goes above 70 if this current limit is increased

    inline constexpr units::turns_per_second_t kMaxAngularVelocity(100); // fixme(MRT) - looks like max velocity practically 70 tps
    // fixme(MRT) - update min angular velocity after tuning 
    //            - (this needs to be lower than actual min angular velocity to give SOTM some margin to vary velocity)
    //inline constexpr units::turns_per_second_t kMinAngularVelocity(37.5);
    inline constexpr units::turns_per_second_t kMinAngularVelocity(20);

    inline constexpr double kFlywheelVelScalingIncrement(0.05); // 5% scaling of flywheel velocity
    inline constexpr double kDefaultFlywheelVelocityScaling(1.0); // tested on practice field, works with min angle (fixme - properly tune robot)
    inline constexpr double kMinFlywheelVelocityScaling(0.5); // allow flywheel speed to be reduced to 50% of nominal
    inline constexpr double kMaxFlywheelVelocityScaling(1.5); // allow flywheel velocity to be increased to 150% of nominal

    inline constexpr units::meter_t kRangeThreshold = 2.0_m; // threshold for changing between min (high range) and max (low range) angle

    inline constexpr units::second_t kRampTime = 0.3_s; // time from stopped to ready for shooting

    
};