// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ExampleSubsystem.h"
#include <subsystems/DriveSubsystem.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/FlyWheelSubsystem.h>
#include <subsystems/HoodSubsystem.h>

namespace autos {

void LoadTrajectories(); // to be called during RobotContainer init BEFORE using any of the below functions

/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);

frc2::CommandPtr ShootCommand(FlyWheelSubsystem* flyWheel, FeederSubsystem* feeder);
frc2::CommandPtr ShootCommand(FlyWheelSubsystem* flyWheel, FeederSubsystem* feeder, HoodSubsystem* hood);
frc2::CommandPtr ShootCommand(FlyWheelSubsystem* flyWheel, FeederSubsystem* feeder, units::second_t feedTime);

frc2::CommandPtr ChoreoAuto(DriveSubsystem* drive, choreo::Trajectory<choreo::SwerveSample>& choreoTraj);

// frc2::CommandPtr ChoreoAutoTest(DriveSubsystem* drive);

// frc2::CommandPtr ChoreoAutoPlan1(DriveSubsystem* drive);

frc2::CommandPtr ChoreoShootTrench(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, FlyWheelSubsystem* flyWheel);

frc2::CommandPtr ChoreoShootFromLeft(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, FlyWheelSubsystem* flyWheel);

frc2::CommandPtr ChoreoShootFromRight(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, FlyWheelSubsystem* flyWheel);

frc2::CommandPtr ChoreoShootFromMiddle(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, FlyWheelSubsystem* flyWheel);

frc2::CommandPtr PlayoffAuto(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, FlyWheelSubsystem* flyWheel, HoodSubsystem* hood);

}  // namespace autos


