#include <subsystems/DriveSubsystem.h>

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