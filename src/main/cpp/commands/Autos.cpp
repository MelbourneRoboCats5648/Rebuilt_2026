// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <choreo/Choreo.h>

#include "commands/ExampleCommand.h"

// pre-loaded trajectories - static so that they are local to this file ONLY
static choreo::Trajectory<choreo::SwerveSample> ShootTrench_Shoot;
static choreo::Trajectory<choreo::SwerveSample> ShootTrench_UnderTrench;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromLeft;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromRight;
static choreo::Trajectory<choreo::SwerveSample> Shoot_fromMiddle;
static choreo::Trajectory<choreo::SwerveSample> Playoff_InitToShoot;
static choreo::Trajectory<choreo::SwerveSample> Playoff_ShootToTower;

void autos::LoadTrajectories() {
    ShootTrench_Shoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path1").value();
    ShootTrench_UnderTrench = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path2").value();
    Shoot_fromLeft = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromLeft").value();
    Shoot_fromMiddle = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromMiddle").value();
    Shoot_fromRight= choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Shoot_fromRight").value();
    Playoff_InitToShoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Playoff_InitToShoot").value();
    Playoff_ShootToTower = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_Playoff_ShootToTower").value();
}

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::ShootCommand(ShooterSubsystem* shooter, FeederSubsystem* feeder) {
    return frc2::cmd::Parallel(
        shooter->ShootCommand(),
        frc2::cmd::Wait(ShooterConstants::kRampTime).AndThen(feeder->FeedCommand())
    );
}

frc2::CommandPtr autos::ShootCommand(ShooterSubsystem* shooter, FeederSubsystem* feeder, HoodSubsystem* hood) {
    return frc2::cmd::Parallel(
        shooter->ShootCommand(),
        hood->GoToAngleCommand(),
        frc2::cmd::Wait(ShooterConstants::kRampTime).AndThen(feeder->FeedCommand())
    );
}

frc2::CommandPtr autos::ShootCommand(ShooterSubsystem* shooter, FeederSubsystem* feeder, units::second_t feedTime) {
    return ShootCommand(shooter, feeder).WithTimeout(ShooterConstants::kRampTime + feedTime);
}

frc2::CommandPtr autos::ChoreoAuto(DriveSubsystem* drive, choreo::Trajectory<choreo::SwerveSample>& choreoTraj) {
    return drive->FollowTrajectoryCommand(choreoTraj);
}

frc2::CommandPtr autos::ChoreoShootTrench(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, ShootTrench_Shoot),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        ShootCommand(shooter, feeder, 5_s),
        ChoreoAuto(drive, ShootTrench_UnderTrench)
    );
}

frc2::CommandPtr autos::ChoreoShootFromLeft(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromLeft),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        ShootCommand(shooter, feeder, 7_s)
    );
}

frc2::CommandPtr autos::ChoreoShootFromRight(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromRight),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        ShootCommand(shooter, feeder, 7_s)
    );
}

frc2::CommandPtr autos::ChoreoShootFromMiddle(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Shoot_fromMiddle),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        ShootCommand(shooter, feeder, 7_s)
    );
}

frc2::CommandPtr autos::PlayoffAuto(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter, HoodSubsystem* hood) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            frc2::cmd::Wait(10_s),
            hood->RetractToLimitCommand()
        ),
        ChoreoAuto(drive, Playoff_InitToShoot),
        intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit), // too risky to extend while moving out (risk of smashing intake)
        ShootCommand(shooter, feeder, 5_s),
        ChoreoAuto(drive, Playoff_ShootToTower)
    );
}