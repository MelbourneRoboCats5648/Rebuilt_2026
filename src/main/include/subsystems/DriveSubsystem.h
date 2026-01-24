#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <drive/DriveModule.h>

#include <constants/HardwareConstants.h>
#include <constants/DriveConstants.h>

#include <units/velocity.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
// #include <frc/controller/HolonomicDriveController.h> // TODO: do something about this
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/trajectory/Trajectory.h>

using namespace ctre::phoenix6::hardware;
using namespace units::velocity;

class DriveSubsystem : public frc2::SubsystemBase {
public:
    /* constructor */
    DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    /* gyroscope */
    void ResetGyro();
    degree_t GetHeading();

    /* kinematics/"set speed" */
    void Drive(
        meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed,
        bool fieldRelative
    );
    void Stop();
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);
    frc::SwerveDriveKinematics<4>& GetKinematics();

    /* odometry/pose estimation */
    frc::SwerveDrivePoseEstimator<4>& GetPoseEstimator();
    frc::Pose2d GetPose();
    void ResetPose(frc::Pose2d pose);

    frc::Trajectory CreateTrajectory(frc::Pose2d targetPose);

private:
    Pigeon2 m_gyro{HardwareConstants::kGyroID, "rio"};

    DriveModule m_frontLeftModule{
        HardwareConstants::kFrontLeftSpeedID, HardwareConstants::kFrontLeftDirectionID, HardwareConstants::kFrontLeftEncoderID,
        DriveModuleConstants::DirectionEncoder::kFrontLeftOffset
    };

    DriveModule m_frontRightModule{
        HardwareConstants::kFrontRightSpeedID, HardwareConstants::kFrontRightDirectionID, HardwareConstants::kFrontRightEncoderID,
        DriveModuleConstants::DirectionEncoder::kFrontRightOffset
    };

    DriveModule m_backLeftModule{
        HardwareConstants::kBackLeftSpeedID, HardwareConstants::kBackLeftDirectionID, HardwareConstants::kBackLeftEncoderID,
        DriveModuleConstants::DirectionEncoder::kBackLeftOffset
    };

    DriveModule m_backRightModule{
        HardwareConstants::kBackRightSpeedID, HardwareConstants::kBackRightDirectionID, HardwareConstants::kBackRightEncoderID,
        DriveModuleConstants::DirectionEncoder::kBackRightOffset
    };

    frc::SwerveDriveKinematics<4> m_kinematics{
        DrivetrainConstants::ModuleLocation::kFrontLeft,
        DrivetrainConstants::ModuleLocation::kFrontRight,
        DrivetrainConstants::ModuleLocation::kBackLeft,
        DrivetrainConstants::ModuleLocation::kBackRight
    };
    
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_kinematics,
        frc::Rotation2d{GetHeading()},
        { m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(), m_backLeftModule.GetPosition(), m_backRightModule.GetPosition() },
        frc::Pose2d{}
    };

    nt::StructArrayPublisher<frc::SwerveModuleState> m_statePublisher; 
    nt::StructArrayPublisher<frc::SwerveModuleState> m_commandPublisher; 
    nt::StructPublisher<frc::Pose2d> m_posePublisher;

};
