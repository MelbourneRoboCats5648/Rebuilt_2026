// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "helpers/ChoreoController.h"

#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <frc/geometry/Pose2d.h>

#include <frc/Timer.h>

class ChoreoTrajectoryCommand
    : public frc2::CommandHelper<frc2::Command, ChoreoTrajectoryCommand> {
public:
    explicit ChoreoTrajectoryCommand(DriveSubsystem* drive, ChoreoController& controller, choreo::Trajectory<choreo::SwerveSample>& trajectory);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    DriveSubsystem* m_drive;

    ChoreoController& m_controller;
    choreo::Trajectory<choreo::SwerveSample>& m_trajectory;

    frc::Timer m_timer;

    // for debugging
    nt::StructArrayPublisher<frc::Pose2d> m_trajectoryPublisher;
    nt::StructPublisher<frc::Pose2d> m_posePublisher;
};
