#include <subsystems/DriveSubsystem.h>
#include <frc/TimedRobot.h>

using namespace ctre::phoenix6::configs;

DriveSubsystem::DriveSubsystem() {
    m_statePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/SwerveStates").Publish();
    m_posePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructTopic<frc::Pose2d>("DriveTrain/Pose").Publish();
        
    /* Configure Pigeon2 */
    Pigeon2Configuration toApply{};

    m_gyro.GetConfigurator().Apply(toApply);
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_gyro.GetYaw(), m_gyro.GetGravityVectorZ()); 

    m_gyro.SetYaw(DrivetrainConstants::kInitialGyroAngle, 100_ms); 
}

void DriveSubsystem::Periodic() {

    m_statePublisher.Set(
        std::vector{
            m_frontLeftModule.GetState(),
            m_frontRightModule.GetState(),
            m_backLeftModule.GetState(),
            m_backRightModule.GetState()
        }
    );
    m_poseEstimator.Update(frc::Rotation2d{GetHeading()},
                    {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                    m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});

    m_posePublisher.Set(m_poseEstimator.GetEstimatedPosition());
}

void DriveSubsystem::SimulationPeriodic() {

}

/* gyroscope */
void DriveSubsystem::ResetGyro() {
    m_gyro.Reset();
}

degree_t DriveSubsystem::GetHeading() {
    return m_gyro.GetRotation2d().Degrees();
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
