#pragma once

namespace HoodConstants {
    inline constexpr units::degree_t kMinAngleSoftLimit = 55_deg;
    inline constexpr units::degree_t kMaxAngleSoftLimit = 67_deg;

    inline constexpr units::degree_t kMinAngle = kMinAngleSoftLimit + 0.5_deg; // actual testing shows that the min angle achievable with PID is 55.13 deg due to stiction
    inline constexpr units::degree_t kMaxAngle = kMaxAngleSoftLimit; // max angle achievable with PID is very close to 67 deg

    inline const units::degree_t kMaxAngleRange = kMaxAngleSoftLimit - kMinAngleSoftLimit;
    inline const double kAngleDegreesPerTurn = kMaxAngleRange.value() / 4.2619; // corresponding to an angle range of 12 degrees for 4.2619 turns of the motor shaft

    inline constexpr units::volt_t kCalibrationVoltage = 2.0_V;
    inline constexpr units::degrees_per_second_t kCalibrationVelocityThreshold = 1.5_deg_per_s;
    inline constexpr units::second_t kCalibrationPreTime = 0.1_s;
    inline constexpr units::second_t kCalibrationTimeout = 0.8_s;

    inline constexpr double kP = 0.55;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;

};