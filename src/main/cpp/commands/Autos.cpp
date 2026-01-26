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

frc2::CommandPtr autos::AutoDepot(DriveSubsystem* drive) {
    frc::Pose2d startPose{3.5439321994781494_m, 4.022441864013672_m, 0_rad};
    frc::Pose2d targetPose{2.4197206497192383_m, 4.022441864013672_m, 0_rad};

    frc::Trajectory traj = drive->CreateTrajectory(startPose, targetPose);

    return frc2::cmd::Sequence(

        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{3.5439321994781494_m, 4.022441864013672_m, 0_rad}, frc::Pose2d{2.4197206497192383_m, 4.022441864013672_m, 0_rad})),
        //shoot
        drive->AlignHeadingCommand(2.3377917235925576_rad),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{0.45742592215538025_m, 4.9360432624816895_m, 1.5707963267948966_rad})),
        //intake
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{1.387824296951294_m, 6.8814215660095215_m,-0.9505471899873089_rad})),
        drive->FollowTrajectoryCommand(drive->CreateTrajectory(frc::Pose2d{2.4197206497192383_m, 4.039477825164795_m,0_rad}))
        //shoot

    );


};
    
 