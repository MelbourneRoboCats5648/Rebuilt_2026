#include <subsystems/DriveSubsystem.h>

DriveSubsystem::DriveSubsystem() {

}

void DriveSubsystem::Periodic() {

    m_poseEstimator.Update(frc::Rotation2d{GetHeading()},
                    {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                    m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});

}

void DriveSubsystem::SimulationPeriodic() {

}

/* gyroscope */
void DriveSubsystem::ResetGyro() {

}

degree_t DriveSubsystem::GetHeading() {

}

/* kinematics/"set speed" */
void DriveSubsystem::Drive(
    meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed,
    bool fieldRelative
) {

}

void DriveSubsystem::Stop() {

}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states) {

}

frc::SwerveDriveKinematics<4>& DriveSubsystem::GetKinematics() {

}

/* odometry/pose estimation */
frc::SwerveDrivePoseEstimator<4>& DriveSubsystem::GetPoseEstimator() {
    return m_poseEstimator;
}

frc::Pose2d DriveSubsystem::GetPose() {

    return m_poseEstimator.GetEstimatedPosition();

}

void DriveSubsystem::ResetPose(frc::Pose2d pose) 
{
    m_poseEstimator.ResetPosition(frc::Rotation2d{GetHeading()},
                    {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                    m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()},
                    pose);
}
