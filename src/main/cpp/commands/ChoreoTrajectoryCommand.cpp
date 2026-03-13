#include "commands/ChoreoTrajectoryCommand.h"
#include "constants/DriveConstants.h"

#include <iostream>

#include <frc/DriverStation.h>

ChoreoTrajectoryCommand::ChoreoTrajectoryCommand(DriveSubsystem* drive, ChoreoController& controller, choreo::Trajectory<choreo::SwerveSample>& trajectory)
    : m_drive(drive), m_controller(controller), m_trajectory(trajectory) {
    m_trajectoryPublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::Pose2d>("DriveTrain/Trajectory/Path").Publish();
    m_posePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructTopic<frc::Pose2d>("DriveTrain/Trajectory/TargetPose").Publish();

    this->AddRequirements({ drive });
}

void ChoreoTrajectoryCommand::Initialize() {
    m_controller.Reset();
    m_timer.Restart();
    
    m_isRed = (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed);
    m_drive->GetPoseEstimator().ResetPose(m_trajectory.GetInitialPose(m_isRed).value());
    
    m_trajectoryPublisher.Set(m_trajectory.GetPoses());
}

void ChoreoTrajectoryCommand::Execute() {
    units::second_t elapsed = m_timer.Get();
    // std::cout << "elapsed time: " << elapsed.value() << std::endl;

    choreo::SwerveSample sample = m_trajectory.SampleAt(elapsed).value();
    if (m_isRed) {
        sample = sample.Flipped(); // flip for red alliance
    }
    m_posePublisher.Set(sample.GetPose());

    frc::ChassisSpeeds speed = m_controller.FollowTrajectory(sample, m_drive->GetPose());

    const bool isFieldCentric = true;
    m_drive->Drive(speed.vx, speed.vy, speed.omega, isFieldCentric); // fixme - could overload to allow direct input of chassis speed
}

bool ChoreoTrajectoryCommand::IsFinished() {
    units::second_t elapsed = m_timer.Get();

    bool endTimeReached = elapsed > m_trajectory.GetTotalTime();
    if (!endTimeReached) {
        return false; // trajectory not ended yet
    }

    if (m_controller.AtSetpoint()) {
        std::cout << "at setpoint, off by " << elapsed.value() - m_trajectory.GetTotalTime().value() << " sec" << std::endl;
        return true; // trajectory ended AND we've hit the final setpoint
    }

    if (elapsed > (m_trajectory.GetTotalTime() * (1.0 + DrivetrainConstants::Autonomous::kTrajTimeTolerance))) {
        std::cout << "trajectory timeout" << std::endl;
        return true; // past time tolerance
    }

    return false; // trajectory ended, we're not past time tolerance yet, and we haven't reached setpoint
}

void ChoreoTrajectoryCommand::End(bool interrupted) {
    m_drive->Stop();
    m_controller.Reset();
    m_timer.Stop();
}
