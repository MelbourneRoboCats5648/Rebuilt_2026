// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "constants/OperatorConstants.h"
#include "constants/DriveConstants.h"
#include <constants/ShooterConstants.h>

#include "subsystems/DriveSubsystem.h"
#include <subsystems/ShooterSubsystem.h>

#include <frc/filter/SlewRateLimiter.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    // Replace with CommandPS4Controller or CommandJoystick if needed
    frc2::CommandXboxController m_driverController{
        OperatorConstants::kDriverControllerPort
    };

    // The robot's subsystems are defined here...
    DriveSubsystem m_drive{};
    ShooterSubsystem m_shooter{};

    void ConfigureBindings();

    double PreprocessJoystickInput(double input);

    frc::SlewRateLimiter<units::meters_per_second> m_xLimiter{DrivetrainConstants::kMaxAcceleration};
    frc::SlewRateLimiter<units::meters_per_second> m_yLimiter{DrivetrainConstants::kMaxAcceleration};
    frc::SlewRateLimiter<units::radians_per_second> m_rotLimiter{DrivetrainConstants::kMaxAngularAcceleration};
};
