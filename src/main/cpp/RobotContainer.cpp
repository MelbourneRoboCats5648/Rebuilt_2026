// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include "commands/Autos.h"

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();


    //the auto routines
    frc2::CommandPtr m_autoClimb = autos::AutoClimb(&m_drive);
    frc2::CommandPtr m_autoTesting = autos::AutoTesting(&m_drive);

    //adding commands to the auto chooser
    m_chooser.SetDefaultOption("Climb Auto", m_autoClimb.get());
    m_chooser.AddOption("Testing Auto", m_autoTesting.get());

    //put the chooser on the dashboard
    frc::SmartDashboard::PutData(&m_chooser);
}

double RobotContainer::PreprocessJoystickInput(double input) {
    double inputDeadband = frc::ApplyDeadband(input, OperatorConstants::kDeadband);
    
    /* scaling */
    double sign = (inputDeadband < 0.0) ? -1.0 : 1.0;
    return sign * inputDeadband * inputDeadband;
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here

    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            meters_per_second_t xSpeed = m_xLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetLeftY())
                * DrivetrainConstants::kMaxSpeed
            );
            meters_per_second_t ySpeed = m_yLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetLeftX())
                * DrivetrainConstants::kMaxSpeed
            );
            radians_per_second_t rotSpeed = m_rotLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetRightX())
                * DrivetrainConstants::kMaxAngularSpeed
            );

            m_drive.Drive(xSpeed, ySpeed, rotSpeed, false);
        },
        { &m_drive }
    ));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}

