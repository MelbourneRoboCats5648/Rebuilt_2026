#include <subsystems/DriveSubsystem.h>

using namespace ctre::phoenix6::configs;

DriveSubsystem::DriveSubsystem() {
    /* Configure Pigeon2 */
  Pigeon2Configuration toApply{};

  m_gyro.GetConfigurator().Apply(toApply);
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_gyro.GetYaw(), m_gyro.GetGravityVectorZ()); 
    
  m_gyro.SetYaw(DrivetrainConstants::kInitialGyroAngle, 100_ms); 
}

void DriveSubsystem::Periodic() {

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

}

void DriveSubsystem::Stop() {

}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states) {

}

frc::SwerveDriveKinematics<4>& DriveSubsystem::GetKinematics() {

}

/* odometry/pose estimation */
frc::SwerveDrivePoseEstimator<4>& DriveSubsystem::GetPoseEstimator() {

}

frc::Pose2d DriveSubsystem::GetPose() {

}

void DriveSubsystem::ResetPose(frc::Pose2d pose) {

}