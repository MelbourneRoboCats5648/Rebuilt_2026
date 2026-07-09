#pragma once

#include <ctre/phoenix6/CANBus.hpp>

namespace HardwareConstants {
    inline constexpr ctre::phoenix6::CANBus kPhoenixCAN = ctre::phoenix6::CANBus::RoboRIO();

    inline constexpr int kGyroID = 14;

    // Swerve Motors
    inline constexpr int kFrontLeftSpeedID = 7;
    inline constexpr int kFrontLeftDirectionID = 8;
    inline constexpr int kFrontLeftEncoderID = 13;

    inline constexpr int kFrontRightSpeedID = 5;
    inline constexpr int kFrontRightDirectionID = 6;
    inline constexpr int kFrontRightEncoderID = 12;

    inline constexpr int kBackLeftSpeedID = 1;
    inline constexpr int kBackLeftDirectionID = 2;
    inline constexpr int kBackLeftEncoderID = 10;

    inline constexpr int kBackRightSpeedID = 3;
    inline constexpr int kBackRightDirectionID = 4;
    inline constexpr int kBackRightEncoderID = 11;

    // Shooter Motors
    inline constexpr int kShooterFlywheelID = 15;
    inline constexpr int kShooterFlywheelFollowerID = 16;

    // feeder
    inline constexpr int kShooterFeederID = 20;
    inline constexpr int kShooterLeftSideFeederID = 22;
    inline constexpr int kShooterRightSideFeederID = 25;

    inline constexpr int kShooterHoodID = 24;

    // Intake Motors
    inline constexpr int kExtendRetractMotorID = 19;
    inline constexpr int kFollowerExtendRetractMotorID = 17;
    inline constexpr int kIntakeMotorID = 23;

};