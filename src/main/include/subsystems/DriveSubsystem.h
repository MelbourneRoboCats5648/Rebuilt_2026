#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <helpers/DriveModule.h>

#include <constants/HardwareConstants.h>
#include <constants/DriveConstants.h>

#include <units/velocity.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <helpers/ChoreoController.h>

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/trajectory/Trajectory.h>

#include "constants/FieldConstants.h"

using namespace ctre::phoenix6::hardware;
using namespace units::velocity;
using namespace DrivetrainConstants;

class DriveSubsystem : public frc2::SubsystemBase {
public:
    /* constructor */
    DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    /* gyroscope */
    void ResetGyro();
    degree_t GetGyroHeading();

    degree_t GetHeading(); // get heading from pose estimator    

    /* kinematics/"set speed" */
    void Drive(
        meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed,
        bool teleop = true // if true, this will invert the field-relative heading if we're on the red alliance
    );

    void Drive(
        meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed,
        bool isfieldRelative, bool teleop
    );
    void Stop();
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);
    frc::SwerveDriveKinematics<4>& GetKinematics();
    frc::ChassisSpeeds GetVelocity();

    frc2::CommandPtr ToggleFieldRelativeCommand();

    /* odometry/pose estimation */
    frc::SwerveDrivePoseEstimator<4>& GetPoseEstimator();
    frc::Pose2d GetPose();
    void ResetPose(frc::Pose2d pose);
    void ResetHeading(degree_t heading);
    void ResetHeadingWithAlliance();

    bool IsFieldCentric();

    frc::Trajectory CreateTrajectory(frc::Pose2d targetPose);
    frc::Trajectory CreateTrajectory(frc::Pose2d currentPose, frc::Pose2d targetPose);
    frc2::CommandPtr FollowTrajectoryCommand(frc::Trajectory trajectory);
    frc2::CommandPtr FollowTrajectoryCommand(choreo::Trajectory<choreo::SwerveSample>& trajectory);

    frc2::CommandPtr AlignHeadingCommand(std::function<radian_t()> headingLambda);
    frc2::CommandPtr AlignHeadingCommand(radian_t heading);
    frc2::CommandPtr AlignToTargetCommand();

    units::radian_t HeadingToTarget(); // could be made private, but seems like a useful public function
    units::meter_t DistanceToTarget();

private:
    bool IsBlueAlliance();
    Pigeon2 m_gyro{HardwareConstants::kGyroID, HardwareConstants::kPhoenixCAN};

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
        ModuleLocation::kFrontLeft,
        ModuleLocation::kFrontRight,
        ModuleLocation::kBackLeft,
        ModuleLocation::kBackRight
    };
    
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_kinematics,
        frc::Rotation2d{GetHeading()},
        { m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(), m_backLeftModule.GetPosition(), m_backRightModule.GetPosition() },
        frc::Pose2d{}
    };

    frc::ProfiledPIDController<units::radians> m_thetaController{
        Autonomous::ThetaController::kP,
        Autonomous::ThetaController::kI,
        Autonomous::ThetaController::kD,
        frc::TrapezoidProfile<units::radian>::Constraints{
            kMaxAngularSpeed, kMaxAngularAcceleration
        }
    };

    frc::HolonomicDriveController m_holonomicController{
        frc::PIDController{
            Autonomous::XYController::kP,
            Autonomous::XYController::kI,
            Autonomous::XYController::kD
        },
        frc::PIDController{
            Autonomous::XYController::kP,
            Autonomous::XYController::kI,
            Autonomous::XYController::kD
        },
        m_thetaController
    };

    ChoreoController m_choreoController;

    nt::StructArrayPublisher<frc::SwerveModuleState> m_statePublisher; 
    nt::StructArrayPublisher<frc::SwerveModuleState> m_commandPublisher; 
    nt::StructPublisher<frc::Pose2d> m_posePublisher;
    nt::StructArrayPublisher<frc::Pose2d> m_trajectoryPublisher;

    bool m_isFieldRelative = true;

    frc::Translation2d m_targetPosition = FieldConstants::kBlueHubPosition;

};
