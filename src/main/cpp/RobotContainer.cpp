// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <choreo/trajectory/Trajectory.h>

#include "commands/Autos.h"
#include "units/math.h"
#include "units/voltage.h"

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();

    // load autonomous trajectories and commands
    autos::LoadTrajectories();
    m_autoNone = frc2::cmd::RunOnce([this] {
        m_drive.ResetHeadingWithAlliance();
    });
    m_SCR_ShootTrench = autos::ChoreoShootTrench(&m_drive, &m_intake, &m_feeder, &m_shooter);
    m_SCR_ShootFromLeft = autos::ChoreoShootFromLeft(&m_drive, &m_intake, &m_feeder, &m_shooter);
    m_SCR_ShootFromRight = autos::ChoreoShootFromRight(&m_drive, &m_intake, &m_feeder, &m_shooter);
    m_SCR_ShootFromMiddle = autos::ChoreoShootFromMiddle(&m_drive, &m_intake, &m_feeder, &m_shooter);


    //adding commands to the auto chooser
    m_chooser.SetDefaultOption("No Autonomous", m_autoNone.value().get());
    m_chooser.AddOption("Choreo Shoot Trench", m_SCR_ShootTrench.value().get());
    m_chooser.AddOption("Choreo Shoot from Left",m_SCR_ShootFromLeft.value().get());
    m_chooser.AddOption("Choreo Shoot from Middle", m_SCR_ShootFromMiddle.value().get());
    m_chooser.AddOption("Choreo Shoot from Right", m_SCR_ShootFromRight.value().get());

    //put the chooser on the dashboard
    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
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

            m_drive.Drive(xSpeed, ySpeed, rotSpeed);
        },
        { &m_drive }
    ));

    // m_driverController.Y().WhileTrue(m_intake.IntakeCommand(50_tps)); // 3000 RPM
    
    m_mechController.LeftBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kRetractSoftLimit));
    m_mechController.RightBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kExtendSoftLimit));

    m_mechController.A().WhileTrue(m_intake.IntakeCommand());

    // m_mechController.POVUp().OnTrue(m_shooter.IncreaseFlywheelVelocity());
    // m_mechController.POVDown().OnTrue(m_shooter.DecreaseFlywheelVelocity());
    // m_mechController.POVLeft().OnTrue(m_shooter.ResetFlywheelVelocity());

    // m_mechController.LeftTrigger().WhileTrue(m_climb.ExtendCommand());
    // m_mechController.RightTrigger().WhileTrue(m_climb.RetractCommand());

   
    m_shooter.SetDefaultCommand(frc2::RunCommand(
        [this] {
            units::degree_t angle;
            angle = (PreprocessJoystickInput(-m_mechController.GetLeftY()) / 2.0 + 0.5) // idle at halfway - move up to increase, move down to decrease shooting angle
                            * ShooterConstants::kMaxAngleRange + ShooterConstants::kMinAngle;
            m_shooter.GoToAngle(angle);

            units::turns_per_second_t angularVelocity;
            angularVelocity = PreprocessJoystickInput(-m_mechController.GetRightY())
                            * ShooterConstants::kMaxAngularVelocity;
            angularVelocity = units::math::abs(angularVelocity);
            m_shooter.ShootAngularVelocity(angularVelocity);
        },
        { &m_shooter }
    ));

    // m_shooter.SetDefaultCommand(frc2::RunCommand(
    //     [this] {
    //         units::degree_t angle;
    //         angle = (PreprocessJoystickInput(-m_driverController.GetRightY()) / 2.0 + 0.5) // idle at halfway - move up to increase, move down to decrease shooting angle
    //                         * ShooterConstants::kMaxAngleRange + ShooterConstants::kMinAngle;
    //         m_shooter.GoToAngle(angle);

    //         units::meter_t distanceToTarget = m_drive.DistanceToTarget();
    //         units::turns_per_second_t flyWheelSpeed = m_shooter.CalculateFlyWheelSpeed(distanceToTarget, angle);
    //         m_shooter.ShootAngularVelocity(flyWheelSpeed);
    //     },
    //     { &m_shooter }
    // ));

    //m_mechController.X().WhileTrue(m_shooter.GoToAngleCommand(ShooterConstants::kMinAngle).Repeatedly());
    //m_mechController.Y().WhileTrue(m_shooter.GoToAngleCommand(ShooterConstants::kMaxAngle).Repeatedly());

    //m_driverController.POVUp().WhileTrue(m_feeder.FeedCommand());

    m_driverController.Y().OnTrue(m_drive.ToggleFieldRelativeCommand());

    m_driverController.LeftTrigger().WhileTrue(m_drive.AlignToTargetCommand());
    m_driverController.RightTrigger().WhileTrue(autos::ShootCommand(&m_shooter, &m_feeder));

    //m_driverController.RightTrigger().WhileTrue(m_drive.AlignToTargetCommand().
    //                                AndThen(m_feeder.FeedCommand()));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}
