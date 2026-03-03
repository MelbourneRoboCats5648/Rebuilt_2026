// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>

//FIXME not sure which are needed
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/filter/SlewRateLimiter.h>

RobotContainer::RobotContainer() 
     :m_elevatorSubsystem(),
        m_armSubsystem(),
        m_elevatorAndArmSubsystem(m_elevatorSubsystem, m_armSubsystem)
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();


    //FIXME not the neatest thing, cant lie idrk what's happenign here.
      /* elevator velocity control override */
  frc2::Trigger elevatorOverrideTrigger([this] { return std::abs(GetMechLeftY()) > General::kFloatTolerance; });
  elevatorOverrideTrigger.WhileTrue(
    /* control 2nd stage */
    frc2::RunCommand([this] {
      m_elevatorSubsystem.m_secondStage.VelocityControl(-GetMechLeftY() * ElevatorConstants::kManualMaxVelocity);
    }, { &m_elevatorSubsystem.m_secondStage }).ToPtr()
    .Until([this] { return m_elevatorSubsystem.m_secondStage.GetHeight() >= ElevatorConstants::kMaxSecondStageHeight; }) // FIXME: this might not work if we attempt to pull 2nd stage down while it's at max height

    /* control 1st stage */
    .AndThen(
      frc2::RunCommand([this] {
        m_elevatorSubsystem.m_secondStage.VelocityControl(-GetMechLeftY() * ElevatorConstants::kManualMaxVelocity);
      }, { &m_elevatorSubsystem.m_firstStage }).ToPtr()
      .Until([this] { return m_elevatorSubsystem.m_firstStage.GetHeight() <= ElevatorConstants::retractSoftLimitFirstStage + ElevatorConstants::kManualRetractLimitTolerance; })
    )
  );

  /* arm velocity control override */
  frc2::Trigger armOverrideTrigger([this] { return std::abs(GetMechRightY()) > General::kFloatTolerance; });
  armOverrideTrigger.WhileTrue(frc2::RunCommand([this] {
    m_armSubsystem.VelocityControl(-ScaleJoystickInput(GetMechRightY()) * ArmConstants::kManualMaxVelocity); // so that up makes the arm go up - TODO: check if we want to scale joystick input here
  }, { &m_armSubsystem }).ToPtr());
}


double RobotContainer::GetMechLeftY() {
  return frc::ApplyDeadband(m_mechController.GetLeftY(), OperatorConstants::kMechDeadband);
}

double RobotContainer::GetMechRightY() {
  return frc::ApplyDeadband(m_mechController.GetRightY(), OperatorConstants::kMechDeadband);
}

//FIXME not sure if this is functionally the same as preprocess or not
double RobotContainer::ScaleJoystickInput(double input) {
  /* no scaling */
  // return input;

  /* square scaling */
  double sign = (input < 0.0) ? -1.0 : 1.0; // since we'll be losing the sign when we square
  return sign * input * input;

  /* cube scaling */
  // return input * input * input;
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

    m_driverController.RightTrigger().OnTrue(m_drive.ToggleFieldRelativeCommand());

    // mech controller bindings
    m_mechController.A().OnTrue(m_elevatorAndArmSubsystem.CollectCoral());
    m_mechController.X().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L1));
    m_mechController.Y().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L2));
    m_mechController.B().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L3));
    m_mechController.RightTrigger().OnTrue(m_elevatorAndArmSubsystem.PlaceCoral());
    m_mechController.LeftTrigger().OnTrue(m_elevatorAndArmSubsystem.DefaultPositionCommand());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous

}
