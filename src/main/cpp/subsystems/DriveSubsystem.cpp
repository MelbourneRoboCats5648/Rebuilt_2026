#include <subsystems/DriveSubsystem.h>
#include <frc/TimedRobot.h>

DriveSubsystem::DriveSubsystem() {

}

void DriveSubsystem::Periodic() {

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
    auto states =
       m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
           fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                               xSpeed, ySpeed, rotSpeed, frc::Rotation2d{GetHeading()})
                         : frc::ChassisSpeeds{xSpeed, ySpeed, rotSpeed},
           frc::TimedRobot::kDefaultPeriod));

    SetModuleStates(states);

}

void DriveSubsystem::Stop() {
    m_frontLeftModule.StopMotors();
    m_frontRightModule.StopMotors();
    m_backLeftModule.StopMotors();
    m_backRightModule.StopMotors();

}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states) {
   
    m_kinematics.DesaturateWheelSpeeds(&states,
                                         DrivetrainConstants::kMaxSpeed);

  m_frontLeftModule.SetState(states[0]);
  m_frontRightModule.SetState(states[1]);
  m_backLeftModule.SetState(states[2]);
  m_backRightModule.SetState(states[3]); 
}

frc::SwerveDriveKinematics<4>& DriveSubsystem::GetKinematics() {
    return m_kinematics;
}

/* odometry/pose estimation */
frc::SwerveDrivePoseEstimator<4>& DriveSubsystem::GetPoseEstimator() {

}

frc::Pose2d DriveSubsystem::GetPose() {

}

void DriveSubsystem::ResetPose(frc::Pose2d pose) {

}