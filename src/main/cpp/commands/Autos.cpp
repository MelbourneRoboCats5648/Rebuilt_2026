// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <choreo/Choreo.h>

#include "commands/ExampleCommand.h"

// pre-loaded trajectories - static so that they are local to this file ONLY
static choreo::Trajectory<choreo::SwerveSample> TestPath;
static choreo::Trajectory<choreo::SwerveSample> Test_Path1;
static choreo::Trajectory<choreo::SwerveSample> Test_Path2;
static choreo::Trajectory<choreo::SwerveSample> Plan1_InitToShoot;
static choreo::Trajectory<choreo::SwerveSample> Plan1_ShootToCollect;
static choreo::Trajectory<choreo::SwerveSample> Plan1_CollectToShoot;
static choreo::Trajectory<choreo::SwerveSample> Plan2_Shoot;
static choreo::Trajectory<choreo::SwerveSample> Plan2_UnderTrench;

void autos::LoadTrajectories() {    
    TestPath = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("TestPath").value();
    Test_Path1 = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Path1").value();
    Test_Path2 = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Test_Path2").value();
    Plan1_InitToShoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Plan1_InitToShoot").value();
    Plan1_ShootToCollect = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Plan1_ShootToCollect").value();
    Plan1_CollectToShoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Plan1_CollectToShoot").value();
    Plan2_Shoot = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path1").value();
    Plan2_UnderTrench = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("SCR_ShootTrench_Path2").value();
}

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::AutoTesting(DriveSubsystem* drive) {
    frc::Pose2d startPose{0_m, 0_m, 90_deg};
    frc::Pose2d targetPose{2_m, 0_m, 0_deg};
    frc::Trajectory traj = drive->CreateTrajectory(startPose, targetPose);

    return drive->AlignHeadingCommand(90_deg).AndThen(drive->FollowTrajectoryCommand(traj));
}

frc2::CommandPtr autos::AutoTesting2(DriveSubsystem* drive) {
    frc::Pose2d startPose{0_m, 0_m, 90_deg};
    frc::Pose2d targetPose{2_m, 0_m, 0_deg};
    frc::Trajectory traj = drive->CreateTrajectory(startPose, targetPose);

    return frc2::cmd::Sequence(
        drive->AlignHeadingCommand(90_deg),
        drive->FollowTrajectoryCommand(traj)
    );
}


frc2::CommandPtr autos::AutoClimb(DriveSubsystem* drive){
    return frc2::cmd::Sequence(
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{12.988293647766113_m, 3.625459671020508_m, -3.141592653589793_rad}, frc::Pose2d{14.201175689697266_m, 3.612825393676758_m, -3.141592653589793_rad})),
        //shoot
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{15.047666549682617_m, 3.612825393676758_m, -3.141592653589793_rad}))
        //climb
    );
}

frc2::CommandPtr autos::ChoreoAuto(DriveSubsystem* drive, choreo::Trajectory<choreo::SwerveSample>& choreoTraj) {
    
    //choreo::Trajectory<choreo::SwerveSample> temporaryTrajectory;

    return drive->FollowTrajectoryCommand(choreoTraj);
}

frc2::CommandPtr autos::ChoreoAutoTest(DriveSubsystem* drive) {
    return frc2::cmd::Sequence(
        ChoreoAuto(drive, Test_Path1),
        frc2::cmd::Wait(5_s),
        ChoreoAuto(drive, Test_Path2)
    );
}

frc2::CommandPtr autos::ChoreoAutoPlan1(DriveSubsystem* drive) {
    return frc2::cmd::Sequence(
        ChoreoAuto(drive, Plan1_InitToShoot),
        // shoot
        ChoreoAuto(drive, Plan1_ShootToCollect)
        // collect
        // so on...
    );
}

frc2::CommandPtr autos::ChoreoShootTrench(DriveSubsystem* drive, IntakeSubsystem* intake, FeederSubsystem* feeder, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            ChoreoAuto(drive, Plan2_Shoot),
            intake->ExtendRetractCommand(IntakeConstants::kExtendSoftLimit)
        ),
        feeder->FeedCommand().WithTimeout(5_s),
        ChoreoAuto(drive, Plan2_UnderTrench)  
    );
}

frc2::CommandPtr autos::AutoDepot(DriveSubsystem* drive) {
    frc::Pose2d startPose{3.5439321994781494_m, 4.022441864013672_m, 0_rad};
    frc::Pose2d targetPose{2.4197206497192383_m, 4.022441864013672_m, 0_rad};

    frc::Trajectory traj = drive->CreateTrajectory(startPose, targetPose);

    return frc2::cmd::Sequence(

        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{3.54_m, 4.02_m, 0_rad}, frc::Pose2d{2.42_m, 4.02_m, 0_rad})),
        //shoot
        drive->AlignHeadingCommand(2.34_rad),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{0.46_m, 4.94_m, 1.57_rad})),
        //intake
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{1.39_m, 6.88_m, -0.95_rad})),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{2.43_m, 4.04_m,0_rad}))
        //shoot

    );
}

frc2::CommandPtr autos::AutoNeutralCollect(DriveSubsystem* drive) {

    return frc2::cmd::Sequence(
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{12.958328247070312_m, 3.980616807937622_m, 3.141592653589793_rad}, frc::Pose2d{14.186327934265137_m, 4.008525848388672_m, 3.141592653589793_rad})),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{14.25610065460205_m, 0.6873465180397034_m, 3.1149323264619024_rad})),
        //shoot->ShootToHubCommand()
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{9.888330459594727_m, 1.3432096242904663_m, 1.5707963267948966_rad})),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{8.939422607421875_m, 2.850299596786499_m, 1.5707963267948966_rad})),
        //intake->ExtendIntake()
        //intake->IntakeBalls()
        drive->AlignHeadingCommand(180_deg)
        //shoot->ShootToAllianceCommand()
    );
}
