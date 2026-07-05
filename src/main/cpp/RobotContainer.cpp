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

#include <iostream>

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();

    // load autonomous trajectories and commands
    autos::LoadTrajectories();
    m_autoNone = frc2::cmd::RunOnce([this] {
        m_drive.ResetHeadingWithAlliance();
    });
    m_SCR_ShootTrench = autos::ChoreoShootTrench(&m_drive, &m_intake, &m_shooter);
    m_SCR_ShootFromLeft = autos::ChoreoShootFromLeft(&m_drive, &m_intake, &m_shooter);
    m_SCR_ShootFromRight = autos::ChoreoShootFromRight(&m_drive, &m_intake, &m_shooter);
    m_SCR_ShootFromMiddle = autos::ChoreoShootFromMiddle(&m_drive, &m_intake, &m_shooter);
    m_SCR_PlayoffAuto = autos::PlayoffAuto(&m_drive, &m_intake, &m_shooter);

    //adding commands to the auto chooser
    m_chooser.SetDefaultOption("No Autonomous", m_autoNone.value().get());
    m_chooser.AddOption("Playoff Auto", m_SCR_PlayoffAuto.value().get());
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

            if (m_invertControls) {
                xSpeed *= -1.0;
                ySpeed *= -1.0;
            }

            radians_per_second_t rotSpeed = m_rotLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetRightX())
                * DrivetrainConstants::kMaxAngularSpeed
            );

            m_drive.Drive(xSpeed, ySpeed, rotSpeed);

            if (m_drive.IsFieldCentric()) {
                std::cout << "FIELD RELATIVE    ";
            } else {
                std::cout << "ROBOT RELATIVE    ";
            }
            if (m_invertControls) {
                std::cout << "JOYSTICK INVERTED";
            } else {
                std::cout << "JOYSTICK NORMAL";
            }
            std::cout << std::endl;
        },
        { &m_drive }
    ));

    m_driverController.B().OnTrue(frc2::cmd::RunOnce([this] {
        m_invertControls = !m_invertControls;
    }));

    m_driverController.Y().OnTrue(m_drive.ToggleFieldRelativeCommand());
 
    std::function<meters_per_second_t()> xSpeedLambda = [this] {
        meters_per_second_t xSpeed = m_xLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetLeftY())
                * DrivetrainConstants::kMaxSpeed
            );
        
            if (m_invertControls) {
                xSpeed *= -1.0;
            }
            return xSpeed;
        };

    std::function<meters_per_second_t()> ySpeedLambda = [this] {
        meters_per_second_t ySpeed = m_yLimiter.Calculate(
                PreprocessJoystickInput(-m_driverController.GetLeftX())
                 * DrivetrainConstants::kMaxSpeed
            );
            if (m_invertControls) {
                ySpeed *= -1.0;
            }
            return ySpeed;
        };

    m_driverController.LeftTrigger().WhileTrue(m_drive.DriveAlignHeadingCommandWrapper(xSpeedLambda, ySpeedLambda));
    m_driverController.RightTrigger().WhileTrue(m_shooter.ShootCommandWithHood().AlongWith(m_intake.IntakeCommand()));

    // rumble if feeder is stalling 
    frc2::Trigger FuelStuckInFeederTrigger([this]{return m_shooter.IsStalling();});    
    FuelStuckInFeederTrigger.Debounce(2_s, frc::Debouncer::DebounceType::kRising).OnTrue(RumbleControllerCommand());

    // fixme(MRT) - temporarily testing hood calibration function. Can remove
    //m_driverController.A().WhileTrue(m_shooter.RetractHoodToLimitCommand());

   m_mechController.LeftBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kRetractSoftLimit));
   m_mechController.RightBumper().WhileTrue(m_intake.ExtendRetractCommand(IntakeConstants::kExtendSoftLimit));

    // fixme(MRT) - temporarily used to test intake with direct voltage command. Delete after testing
    //m_mechController.LeftBumper().WhileTrue(m_intake.SetExtendRetractVoltageCommand(5_V));
    //m_mechController.RightBumper().WhileTrue(m_intake.SetExtendRetractVoltageCommand(-5_V));

    // fixme(MRT) - need to uncomment the intake command
    m_mechController.RightTrigger().WhileTrue(m_intake.IntakeCommand());
    // m_driverController.Y().WhileTrue(m_intake.IntakeCommand(50_tps)); // 3000 RPM

    //m_mechController.X().OnTrue(m_shooter.SetHoodTargetAngleCommand(HoodConstants::kMinAngle));
    //m_mechController.B().OnTrue(m_shooter.SetHoodTargetAngleCommand((HoodConstants::kMinAngle + HoodConstants::kMidAngle) / 2)); // 1/4 to min
    //m_mechController.Y().OnTrue(m_shooter.SetHoodTargetAngleCommand(HoodConstants::kMaxAngle));
    //m_mechController.X().OnTrue(m_shooter.SetHoodTargetAngleCommand(HoodConstants::kMidAngle));

//    m_mechController.POVUp().OnTrue(m_shooter.IncreaseFeederVoltageDifference());
//    m_mechController.POVDown().OnTrue(m_shooter.DecreaseFeederVoltageDifference());

    // fixme(MRT) - uncomment to allow tuning on flywheel velocity for competition
    // m_mechController.POVUp().OnTrue(m_shooter.IncreaseFlywheelVelocity());
    // m_mechController.POVDown().OnTrue(m_shooter.DecreaseFlywheelVelocity());
    // m_mechController.POVLeft().OnTrue(m_shooter.ResetFlywheelVelocity());

    // fixme(MRT) - modify some version of this default command to check that static shoot solution works for various input angles
    //            - will be good to test this with April tags so that robot knows accurate distance to hub
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

    // fixme(MRT) - Feed command unlikely to be used alone and could be removed
    //m_driverController.POVUp().WhileTrue(m_feeder.FeedCommand());
    // m_driverController.Y().WhileTrue(m_intake.IntakeCommand(50_tps)); // 3000 RPM

    // fixme(MRT) - remove static AlignToTargetCommand and change LeftTrigger binding to DriveAlignHeadingCommandWrapper
    //m_driverController.LeftTrigger().WhileTrue(m_drive.AlignToTargetCommand());

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}

frc2::CommandPtr RobotContainer::RumbleControllerCommand() {
    return frc2::cmd::RunOnce([this]
                   {m_mechController.SetRumble(frc::GenericHID::kBothRumble, 1.0);})

            .AndThen(frc2::cmd::Wait(0.5_s))

            .AndThen(frc2::cmd::RunOnce([this] {
            m_mechController.SetRumble(frc::GenericHID::kBothRumble, 0.0);
            }));
};
