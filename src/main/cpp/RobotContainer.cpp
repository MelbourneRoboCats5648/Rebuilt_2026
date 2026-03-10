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
    m_autoClimb = autos::AutoClimb(&m_drive);
    m_autoTesting = autos::AutoTesting(&m_drive);
    m_autoDepot = autos::AutoDepot(&m_drive);
    m_autoNeutralCollect = autos::AutoNeutralCollect(&m_drive);
    m_choreoTest = autos::ChoreoAutoTest(&m_drive);
    m_choreoPlan1 = autos::ChoreoAutoPlan1(&m_drive);

    //adding commands to the auto chooser
    m_chooser.AddOption("Testing Auto", m_autoTesting.value().get());
    m_chooser.AddOption("Climb Auto", m_autoClimb.value().get());
    m_chooser.AddOption("Depot Auto", m_autoDepot.value().get());
    m_chooser.AddOption("Auto Neutral Collect", m_autoNeutralCollect.value().get());
    m_chooser.AddOption("Choreo Test", m_choreoTest.value().get());
    m_chooser.AddOption("Choreo Plan 1", m_choreoPlan1.value().get());

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

    m_driverController.X().WhileTrue(m_intake.IntakeCommand()); // should slow down as the robot moves forward
    // m_driverController.Y().WhileTrue(m_intake.IntakeCommand(50_tps)); // 3000 RPM
    
    m_mechController.LeftBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kRetractSoftLimit));
    m_mechController.RightBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kExtendSoftLimit));

    m_driverController.A().OnTrue(m_drive.ToggleFieldRelativeCommand());
   
    // m_shooter.SetDefaultCommand(frc2::RunCommand(
    //     [this] {
    //         units::turns_per_second_t angularVelocity;
    //         angularVelocity = PreprocessJoystickInput(-m_driverController.GetRightY())
    //                         * ShooterConstants::kMaxAngularVelocity;
    //         m_shooter.ShootAngularVelocity(angularVelocity);
    //     },
    //     { &m_shooter }
    // ));

    // m_shooter.SetDefaultCommand(frc2::RunCommand(
    //     [this] {
    //         units::degree_t angle;
    //         angle = (PreprocessJoystickInput(-m_driverController.GetRightY()) / 2.0 + 0.5) // idle at halfway - move up to increase, move down to decrease shooting angle
    //                         * ShooterConstants::kMaxAngleRange + ShooterConstants::kMinAngle;
    //         m_shooter.GoToAngle(angle);

    //         units::meter_t distanceToHub = m_shooter.DistanceToHub(m_drive.GetPose());
    //         units::turns_per_second_t flyWheelSpeed = m_shooter.CalculateFlyWheelSpeed(distanceToHub, angle);
    //         m_shooter.ShootAngularVelocity(flyWheelSpeed);
    //     },
    //     { &m_shooter }
    // ));

    m_mechController.X().WhileTrue(m_shooter.GoToAngleCommand(ShooterConstants::kMinAngle).Repeatedly());
    m_mechController.Y().WhileTrue(m_shooter.GoToAngleCommand(ShooterConstants::kMaxAngle).Repeatedly());

    //m_mechController.RightTrigger().WhileTrue(m_climb.ClimbUpCommand());
    //m_mechController.LeftTrigger().WhileTrue(m_climb.ClimbDownCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
    // fixme: consider doing a sequence starting with GetCalibrationCommand()
}

frc2::CommandPtr RobotContainer::GetCalibrationCommand() {
    return frc2::cmd::Either(
        frc2::cmd::None(), // do nothing if m_isCalibrated is true
        frc2::cmd::Parallel(m_shooter.RetractToLimitCommand(), m_intake.RetractToLimitCommand()),
        [this] {
            if (!m_isCalibrated) {
                m_isCalibrated = true; // should be false only once
                return false;
            } else {
                return true;
            }
        }
    );
}