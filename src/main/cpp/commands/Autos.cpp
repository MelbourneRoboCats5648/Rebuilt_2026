// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

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


frc2::CommandPtr autos::ChoreoAuto(DriveSubsystem* drive) {
    
    choreo::Trajectory<choreo::SwerveSample> temporaryTrajectory;

    drive->FollowTrajectoryCommand(temporaryTrajectory);
}