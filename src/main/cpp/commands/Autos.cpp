// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <choreo/Choreo.h>

#include "commands/ExampleCommand.h"

// pre-loaded trajectories - static so that they are local to this file ONLY
// static choreo::Trajectory<choreo::SwerveSample> TestPath;
// static choreo::Trajectory<choreo::SwerveSample> Test_Path1;
// static choreo::Trajectory<choreo::SwerveSample> Test_Path2;
// static choreo::Trajectory<choreo::SwerveSample> Plan1_InitToShoot;
// static choreo::Trajectory<choreo::SwerveSample> Plan1_ShootToCollect;
// static choreo::Trajectory<choreo::SwerveSample> Plan1_CollectToShoot;
static choreo::Trajectory<choreo::SwerveSample> ShootTrench_Shoot;
static choreo::Trajectory<choreo::SwerveSample> ShootTrench_UnderTrench;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromLeft;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromRight;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromMiddle;

void autos::LoadTrajectories() {
    ShootTrench_Shoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path1").value();
    ShootTrench_UnderTrench = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path2").value();
    Shoot_fromLeft = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromLeft").value();
    Shoot_fromMiddle = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromMiddle").value();
    Shoot_fromRight= choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromRight").value();
}

static bool IsCalibrated = false;

frc2::CommandPtr autos::CalibrationCommand(IntakeSubsystem* intake, ShooterSubsystem* shooter) {
    return frc2::cmd::Either(
        frc2::cmd::None(), // do nothing if IsCalibrated is true
        // frc2::cmd::Parallel(shooter->RetractToLimitCommand(), intake->RetractToLimitCommand()),
        shooter->RetractToLimitCommand(),
        [] {
            if (!IsCalibrated) {
                IsCalibrated = true; // should be false only once
                return false;
            } else {
                return true;
            }
        }
    );
}

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::ChoreoAuto(DriveSubsystem* drive, choreo::Trajectory<choreo::SwerveSample>& choreoTraj) {
    return drive->FollowTrajectoryCommand(choreoTraj);
}

frc2::CommandPtr autos::ChoreoShootTrench(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, ShootTrench_Shoot),
            frc2::cmd::Sequence( // calibrate before doing anything
                CalibrationCommand(intake, shooter),
                intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
            )
        ),
        frc2::cmd::Parallel(
            shooter->DefaultShootCommand(),
            frc2::cmd::Wait(1_s).AndThen( // ramp up delay
                frc2::cmd::Sequence(
                    feeder->FeedCommand().WithTimeout(5_s),
                    ChoreoAuto(drive, ShootTrench_UnderTrench)
                )
            )
        )
    );
}

frc2::CommandPtr autos::ChoreoShootFromLeft(DriveSubsystem* drive, FeederSubsystem* feeder, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromLeft),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        feeder->FeedCommand().WithTimeout(7_s)
    );
}

frc2::CommandPtr autos::ChoreoShootFromRight(DriveSubsystem* drive, FeederSubsystem* feeder, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromRight),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        feeder->FeedCommand().WithTimeout(7_s)
    );
}

frc2::CommandPtr autos::ChoreoShootFromMiddle(DriveSubsystem* drive, FeederSubsystem* feeder, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromMiddle),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        feeder->FeedCommand().WithTimeout(7_s)
    );
}