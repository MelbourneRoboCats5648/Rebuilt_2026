#pragma once

namespace HardwareConstants {
    inline constexpr int kGyroID = 14;

    // Swerve Motors
    inline constexpr int kFrontLeftSpeedID = 3;
    inline constexpr int kFrontLeftDirectionID = 4;
    inline constexpr int kFrontLeftEncoderID = 11;

    inline constexpr int kFrontRightSpeedID = 1;
    inline constexpr int kFrontRightDirectionID = 2;
    inline constexpr int kFrontRightEncoderID = 10;

    inline constexpr int kBackLeftSpeedID = 5;
    inline constexpr int kBackLeftDirectionID = 6;
    inline constexpr int kBackLeftEncoderID = 12;

    inline constexpr int kBackRightSpeedID = 7;
    inline constexpr int kBackRightDirectionID = 8;
    inline constexpr int kBackRightEncoderID = 13;

    // Shooter Motors
    inline constexpr int kShooterFlywheelID = 15;
    inline constexpr int kShooterFlywheelFollowerID = 16;

    inline constexpr int kShooterFeederID = 17;
    inline constexpr int kShooterHoodID = 18;

    // Intake Motors
    inline constexpr int kExtendRetractMotorID = 19; 
    inline constexpr int kFollowerExtendRetractMotorID = 20; 
    inline constexpr int kIntakeMotorID = 33; //fixme - can id 

    // Climb Motors
    inline constexpr int kClimbMotorID = 21;
    inline constexpr int kClimbFollowerMotorID = 22;

};