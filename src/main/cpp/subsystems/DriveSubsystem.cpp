#include <subsystems/DriveSubsystem.h>
#include <frc/TimedRobot.h>

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>

using namespace ctre::phoenix6::configs;

DriveSubsystem::DriveSubsystem() {
    m_statePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/SwerveStates").Publish();
    m_commandPublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/CommandedStates").Publish();
    m_posePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructTopic<frc::Pose2d>("DriveTrain/Pose").Publish();
    m_trajectoryPublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructArrayTopic<frc::Pose2d>("DriveTrain/FollowingTrajectory").Publish();    
    
    /* Configure Pigeon2 */
    Pigeon2Configuration toApply{};

    m_gyro.GetConfigurator().Apply(toApply);
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_gyro.GetYaw(), m_gyro.GetGravityVectorZ()); 

    m_gyro.SetYaw(DrivetrainConstants::kInitialGyroAngle, 100_ms); 

    m_holonomicController.SetTolerance(
        frc::Pose2d(
            Autonomous::XYController::kTolerance, Autonomous::XYController::kTolerance, // translation
            Autonomous::ThetaController::kPositionTolerance // rotation
        )
    );
    m_thetaController.SetTolerance(Autonomous::ThetaController::kPositionTolerance, Autonomous::ThetaController::kVelocityTolerance);
    // note that m_thetaController's input range is 0 to 360 deg, not -180 to 180! (set by m_holonomicController)
}

void DriveSubsystem::Periodic() {
    m_poseEstimator.Update(frc::Rotation2d{GetHeading()},
                    {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                    m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});

    /* publish current state */
    m_statePublisher.Set(
        std::vector{
            m_frontLeftModule.GetState(),
            m_frontRightModule.GetState(),
            m_backLeftModule.GetState(),
            m_backRightModule.GetState()
        }
    );
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
    Drive(0_mps, 0_mps, 0_rad_per_s, false);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states) {
    m_kinematics.DesaturateWheelSpeeds(&states, DrivetrainConstants::kMaxSpeed);
    m_commandPublisher.Set(states);

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

frc::Trajectory DriveSubsystem::CreateTrajectory(frc::Pose2d targetPose) {
  return CreateTrajectory(
    m_poseEstimator.GetEstimatedPosition(),
    std::move(targetPose)
  );
}

frc::Trajectory DriveSubsystem::CreateTrajectory(frc::Pose2d currentPose, frc::Pose2d targetPose) {
  frc::TrajectoryConfig config{DrivetrainConstants::kMaxSpeed,
                               DrivetrainConstants::kMaxAcceleration};

  config.SetKinematics(m_kinematics);

  // A trajectory to follow.  All units in meters.
  auto traj = frc::TrajectoryGenerator::GenerateTrajectory(
      std::move(currentPose), //current pose from pose estimatior
      {},
      std::move(targetPose),
      config);

  return traj;
}

  // Reset odometry to the initial pose of the trajectory, run path following command, then stop at the end.
  frc2::CommandPtr DriveSubsystem::FollowTrajectoryCommand(frc::Trajectory trajectory) {
    return RunOnce([this, initialPose = trajectory.InitialPose(), trajectory] {
        m_poseEstimator.ResetPose(initialPose);  //fixme - this may not be required

        /* publish trajectory */
        std::vector<frc::Pose2d> poses;
        std::vector<frc::Trajectory::State> states = trajectory.States();
        for (frc::Trajectory::State& state : states) {
            poses.push_back(state.pose);
        }
        m_trajectoryPublisher.Set(poses);        
    })
    .AndThen(
        frc2::SwerveControllerCommand<4>(
            trajectory, 
            [this] { return GetPose(); },
            m_kinematics,
            m_holonomicController,
            [this](std::array<frc::SwerveModuleState, 4> states) { SetModuleStates(states); }
        ).ToPtr()
    ).AndThen( // keep running controller until we actually reach goal
        Run([this, lastState = trajectory.States().back()] {
            auto desiredSpeed = m_holonomicController.Calculate(GetPose(), lastState, lastState.pose.Rotation());
            auto desiredStates = m_kinematics.ToSwerveModuleStates(desiredSpeed);
            SetModuleStates(desiredStates);
        }).Until([this] {
            return m_holonomicController.AtReference();
        })
    )
    .FinallyDo([this] { this->Stop(); });
}

frc2::CommandPtr DriveSubsystem::FollowTrajectoryCommand(choreo::Trajectory<choreo::SwerveSample> trajectory) {
    return RunOnce([this]{
        radian_t desiredHeading = 0.0_rad;

        m_choreoController.getHeadingController().Reset();
        m_choreoController.getHeadingController().SetSetpoint(desiredHeading.value());
    }).AndThen(
        Run([this] {
            radians_per_second_t angularRate{m_choreoController.getHeadingController()
                                            .Calculate(radian_t{GetHeading()}.value())};
            Drive(0_mps, 0_mps, angularRate, false);
        }).Until([this] {
            return false;
        }))
    .FinallyDo([this]{ }); 
}


frc2::CommandPtr DriveSubsystem::AlignHeadingCommand(std::function<radian_t()> headingLambda) {
    return RunOnce([this, headingLambda] {
        radian_t heading = headingLambda();
        m_thetaController.Reset(GetHeading());
        m_thetaController.SetGoal(heading); // velocity = 0 by default
    }).AndThen(
        Run([this] {
            radians_per_second_t angularRate{m_thetaController.Calculate(GetHeading())};
            Drive(0_mps, 0_mps, angularRate, false);
        }).Until([this] {
            return m_thetaController.AtGoal();
        })
    ).FinallyDo([this] {
        Stop();
        m_thetaController.Reset(GetHeading()); // optional, but just to be safe here
    });
}

frc2::CommandPtr DriveSubsystem::AlignHeadingCommand(radian_t heading) {
    return AlignHeadingCommand([heading] { return heading; });
}