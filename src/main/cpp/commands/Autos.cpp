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

frc2::CommandPtr autos::AutoTestNeutralCollect(DriveSubsystem* drive) {

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