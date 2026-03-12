#pragma once

#include "units/length.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include <units/voltage.h>
#include "FieldConstants.h"
#include <units/current.h>



using namespace units::length;
using namespace units::angle;

namespace ShooterConstants {
    namespace motor {
        inline constexpr double kP = 0.2;
        inline constexpr double kI = 0;
        inline constexpr double kD = 0;
        inline constexpr double kV = 0.22956;
        inline constexpr double kS = 0.087569;
        inline constexpr double kA = 0;
    }

    inline constexpr meter_t startHeight(0.3);
    inline constexpr meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;
    inline constexpr double kgearRatio = 2;

    inline constexpr units::volt_t kMaxVoltage(12);
    inline constexpr units::meter_t kFlyWheelRadius(2_in);

    inline constexpr int kCurrentLimit(50);

    //inline constexpr double kAngleGearRatio = 1.0/5.0;
    inline constexpr units::degree_t kMinAngleSoftLimit = 55_deg;
    inline constexpr units::degree_t kMaxAngleSoftLimit = 67_deg;

    inline constexpr units::degree_t kMinAngle = kMinAngleSoftLimit + 0.5_deg; // actual testing shows that the min angle achievable with PID is 55.13 deg due to stiction
    inline constexpr units::degree_t kMaxAngle = kMaxAngleSoftLimit; // max angle achievable with PID is very close to 67 deg

    inline const units::degree_t kMaxAngleRange = kMaxAngleSoftLimit - kMinAngleSoftLimit;
    inline const double kAngleDegreesPerTurn = kMaxAngleRange.value() / 4.2619; // corresponding to an angle range of 12 degrees for 4.2619 turns of the motor shaft

    namespace angle {
        inline constexpr double kP = 0.55;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;

        inline constexpr units::volt_t kCalibrationVoltage = 2.0_V;
        inline constexpr units::degrees_per_second_t kCalibrationVelocityThreshold = 1.5_deg_per_s;
        inline constexpr units::second_t kCalibrationPreTime = 0.1_s;
        inline constexpr units::second_t kCalibrationTimeout = 0.8_s;
    }

    inline constexpr units::turns_per_second_t kMaxAngularVelocity(50);

    inline constexpr units::volt_t kFeederVoltage(10_V);
    inline constexpr units::volt_t kSideFeederVoltage(5_V); // fixme
};