// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>

#include "commands/Autos.h"
#include "units/math.h"
#include "units/voltage.h"

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();
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

            //rotSpeed = 0_rad_per_s;
            m_drive.Drive(xSpeed, ySpeed, rotSpeed);
        },
        { &m_drive }


    ));

    /*
    m_shooter.SetDefaultCommand(frc2::RunCommand(
        [this]{
                units::volt_t volts;
                volts = PreprocessJoystickInput(-m_driverController.GetRightY()) 
                            * ShooterConstants::kMaxVoltage;

                volts = units::math::abs(volts);

                m_shooter.Shoot(volts);
            },
            { &m_shooter }
        ));
    */
    
    m_driverController.A().OnTrue(m_drive.ToggleFieldRelativeCommand());

    m_shooter.SetDefaultCommand(frc2::RunCommand(
        [this] {
            units::turns_per_second_t angularVelocity;
            angularVelocity = PreprocessJoystickInput(-m_driverController.GetRightY())
                            * ShooterConstants::kMaxAngularVelocity;
            m_shooter.ShootAngularVelocity(angularVelocity);
        },
        { &m_shooter }
    ));

    m_shooter.SetDefaultCommand(frc2::RunCommand(
        [this] {
            units::degree_t shooterAngle = 30_deg;
            units::meter_t distanceToHub = m_shooter.DistanceToHub(m_drive.GetPose());
            units::turns_per_second_t flyWheelSpeed = m_shooter.CalculateFlyWheelSpeed(distanceToHub, shooterAngle);

            m_shooter.ShootAngularVelocity(flyWheelSpeed);
        },
        { &m_shooter }
    ));

    //m_driverController.RightTrigger().WhileTrue(m_climb.ClimbUpCommand());
    //m_driverController.LeftTrigger().WhileTrue(m_climb.ClimbDownCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return autos::AutoTesting2(&m_drive);
}
