#include <subsystems/DriveSubsystem.h>
#include <frc/TimedRobot.h>

#include <commands/ChoreoTrajectoryCommand.h>

#include <frc/DriverStation.h>

using namespace ctre::phoenix6::configs;

DriveSubsystem::DriveSubsystem()
{
    m_statePublisher = nt::NetworkTableInstance::GetDefault()
                           .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/SwerveStates")
                           .Publish();
    m_commandPublisher = nt::NetworkTableInstance::GetDefault()
                             .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/CommandedStates")
                             .Publish();
    m_posePublisher = nt::NetworkTableInstance::GetDefault()
                          .GetStructTopic<frc::Pose2d>("DriveTrain/Pose")
                          .Publish();
    m_alignedPosePublisher = nt::NetworkTableInstance::GetDefault()
                          .GetStructTopic<frc::Pose2d>("DriveTrain/AlignedPose")
                          .Publish();
    m_trajectoryPublisher = nt::NetworkTableInstance::GetDefault()
                                .GetStructArrayTopic<frc::Pose2d>("DriveTrain/FollowingTrajectory")
                                .Publish();

    /* Configure Pigeon2 */
    Pigeon2Configuration toApply{};
    m_gyro.GetConfigurator().Apply(toApply);
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_gyro.GetYaw(), m_gyro.GetGravityVectorZ());

    ResetGyro();
    m_gyro.SetYaw(DrivetrainConstants::kInitialGyroAngle, 100_ms);

    m_thetaController.SetTolerance(Autonomous::ThetaController::kPositionTolerance, Autonomous::ThetaController::kVelocityTolerance);
    // note that m_thetaController's input range is 0 to 360 deg, not -180 to 180!
}

bool DriveSubsystem::IsBlueAlliance()
{
    return (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue);
}

units::radian_t DriveSubsystem::GetShootOnTheMoveHeading() {
    return HeadingToTarget() + GetYawAngle();
}

void DriveSubsystem::Periodic()
{
    m_poseEstimator.Update(frc::Rotation2d{GetGyroHeading()},
                           {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                            m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});

    // Adjust location of target position based on robot location wrt hub
    frc::Translation2d hubPosition =
        (IsBlueAlliance())
            ? FieldConstants::kBlueHubPosition
            : FieldConstants::kRedHubPosition;
    auto xPosition = GetPose().X();
    auto yPosition = GetPose().Y();
    if (
        xPosition > FieldConstants::kBlueHubPosition.X() && xPosition < FieldConstants::kRedHubPosition.X())
    {
        // robot is in neutral zone - aim out of the hub
        if (yPosition < hubPosition.Y())
        { // below centre line
            m_targetPosition = hubPosition - FieldConstants::kNeutralOffset;
        }
        else
        { // above centre line
            m_targetPosition = hubPosition + FieldConstants::kNeutralOffset;
        }
    }
    else
    {
        // robot is in shooting zone - aim towards alliance hub
        m_targetPosition = hubPosition;
    }

    /* publish current state */
    m_statePublisher.Set(
        std::vector{
            m_frontLeftModule.GetState(),
            m_frontRightModule.GetState(),
            m_backLeftModule.GetState(),
            m_backRightModule.GetState()});
    frc::Pose2d pose = m_poseEstimator.GetEstimatedPosition();
    m_posePublisher.Set(pose);

    /* publish aligned pose */
    frc::Pose2d alignedPose(pose.Translation(), frc::Rotation2d(GetShootOnTheMoveHeading()));
    m_alignedPosePublisher.Set(alignedPose);
}

void DriveSubsystem::SimulationPeriodic()
{
}

/* gyroscope */
void DriveSubsystem::ResetGyro()
{
    m_gyro.Reset();
}

degree_t DriveSubsystem::GetGyroHeading()
{
    return m_gyro.GetRotation2d().Degrees();
}

degree_t DriveSubsystem::GetHeading()
{
    return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed, bool teleop)
{
    Drive(xSpeed, ySpeed, rotSpeed, teleop, m_isFieldRelative);
}

bool DriveSubsystem::IsFieldCentric()
{
    return m_isFieldRelative;
}

/* kinematics/"set speed" */
void DriveSubsystem::Drive(
    meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rotSpeed,
    bool teleop, bool fieldRelative)
{
    frc::ChassisSpeeds speeds;
    if (fieldRelative)
    {
        units::radian_t heading = GetHeading();
        if (teleop && !IsBlueAlliance())
        { // flip heading
            heading -= 180_deg;
        }
        speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed,
            frc::Rotation2d{heading});
    }
    else
    { // robot-relative
        speeds = {xSpeed, ySpeed, rotSpeed};
    }

    auto states = m_kinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds::Discretize(speeds, frc::TimedRobot::kDefaultPeriod));

    SetModuleStates(states);
}

void DriveSubsystem::Stop()
{
    Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states)
{
    m_kinematics.DesaturateWheelSpeeds(&states, DrivetrainConstants::kMaxSpeed);
    m_commandPublisher.Set(states);

    m_frontLeftModule.SetState(states[0]);
    m_frontRightModule.SetState(states[1]);
    m_backLeftModule.SetState(states[2]);
    m_backRightModule.SetState(states[3]);
}

frc::SwerveDriveKinematics<4> &DriveSubsystem::GetKinematics()
{
    return m_kinematics;
}

/* odometry/pose estimation */
frc::SwerveDrivePoseEstimator<4> &DriveSubsystem::GetPoseEstimator()
{
    return m_poseEstimator;
}

frc::Pose2d DriveSubsystem::GetPose()
{
    return m_poseEstimator.GetEstimatedPosition();
}

frc::Translation2d DriveSubsystem::GetTargetPosition()
{
    return m_targetPosition;
}

SpeedComponents DriveSubsystem::GetSpeedComponents()
{
    return FindSpeedComponents(GetPose(), GetTargetPosition(), GetVelocity());
}

void DriveSubsystem::SetYawAngle(degree_t shootingYawAngle)
{
    m_shootingYawAngle = shootingYawAngle;
};

degree_t DriveSubsystem::GetYawAngle()
{
    return m_shootingYawAngle;
};

void DriveSubsystem::ResetPose(frc::Pose2d pose)
{
    m_poseEstimator.ResetPosition(frc::Rotation2d{GetGyroHeading()},
                                  {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                                   m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()},
                                  pose);
}

void DriveSubsystem::ResetHeading(degree_t heading)
{
    m_poseEstimator.ResetRotation(frc::Rotation2d{heading});
}

void DriveSubsystem::ResetHeadingWithAlliance()
{
    ResetHeading(
        (IsBlueAlliance())
            ? DrivetrainConstants::kInitialBlueHeading
            : DrivetrainConstants::kInitialRedHeading);
}

frc2::CommandPtr DriveSubsystem::FollowTrajectoryCommand(choreo::Trajectory<choreo::SwerveSample> &trajectory)
{
    return ChoreoTrajectoryCommand(this, m_choreoController, trajectory).ToPtr();
}

frc2::CommandPtr DriveSubsystem::AlignHeadingCommand(std::function<radian_t()> headingLambda)
{
    return RunOnce([this, headingLambda]
                   {
                       radian_t heading = headingLambda();
                       m_thetaController.Reset(GetHeading());
                       m_thetaController.SetGoal(heading); // velocity = 0 by default
                   })
        .AndThen(Run([this]
                     {
            radians_per_second_t angularRate{m_thetaController.Calculate(GetHeading())};
            Drive(0_mps, 0_mps, angularRate, false, false); })
                     .Until([this]
                            { return m_thetaController.AtGoal(); }))
        .FinallyDo([this]
                   {
                       Stop();
                       m_thetaController.Reset(GetHeading()); // optional, but just to be safe here
                   });
}

frc2::CommandPtr DriveSubsystem::DriveAlignHeadingCommand(std::function<radian_t()> headingLambda, std::function<meters_per_second_t()> xSpeedLambda, std::function<meters_per_second_t()> ySpeedLambda)
{
    return RunOnce([this]
                   {
                       m_thetaController.Reset(GetHeading());
                   })
        .AndThen(Run([this, xSpeedLambda, ySpeedLambda, headingLambda] {
            meters_per_second_t x = xSpeedLambda();
            meters_per_second_t y = ySpeedLambda(); 
            m_thetaController.SetGoal(headingLambda());           
            radians_per_second_t angularRate{m_thetaController.Calculate(GetHeading())};
            Drive(x, y, angularRate, true); // true since we're in teleop when we're calling this
        }))
        .FinallyDo([this]
                   {
                       m_thetaController.Reset(GetHeading()); // optional, but just to be safe here
                   });
}

frc2::CommandPtr DriveSubsystem::AlignHeadingCommand(radian_t heading)
{
    std::function<radian_t()> headingLambda = [heading] { return heading; };
    return AlignHeadingCommand(headingLambda);
}

units::radian_t DriveSubsystem::HeadingToTarget()
{
    return (m_targetPosition - GetPose().Translation()).Angle().Radians();
    // this will return the angle that the vector from robot position to target position makes with X+ axis
    // which is the desired heading to turn the robot to
}

frc2::CommandPtr DriveSubsystem::AlignToTargetCommand()
{
    return AlignHeadingCommand([this]
                               { return HeadingToTarget(); });
}

frc2::CommandPtr DriveSubsystem::DriveAlignHeadingCommandWrapper(std::function<meters_per_second_t()> xSpeedLambda, std::function<meters_per_second_t()> ySpeedLambda)
{
    std::function<radian_t()> headingLambda = [this] { return GetShootOnTheMoveHeading(); };
    return DriveAlignHeadingCommand(headingLambda, xSpeedLambda, ySpeedLambda);
}

frc2::CommandPtr DriveSubsystem::ToggleFieldRelativeCommand()
{
    return RunOnce([this]
                   { m_isFieldRelative = !m_isFieldRelative; });
}

frc::ChassisSpeeds DriveSubsystem::GetVelocity()
{
    return m_kinematics.ToChassisSpeeds(
        m_frontLeftModule.GetState(),
        m_frontRightModule.GetState(),
        m_backLeftModule.GetState(),
        m_backRightModule.GetState());
}

units::meter_t DriveSubsystem::DistanceToTarget()
{
    return GetPose().Translation().Distance(m_targetPosition);
}

SpeedComponents DriveSubsystem::FindSpeedComponents(frc::Pose2d robotPose, frc::Translation2d targetPosition, frc::ChassisSpeeds chassisSpeed)
{
    frc::Translation2d vectorRadial = targetPosition - robotPose.Translation();

    units::meter_t magVecToTarget = vectorRadial.Norm();
    frc::Translation2d unitVecRadial = vectorRadial / magVecToTarget.value();
     
    //we need to dot product the unitVector to target with the chassisSpeed vector
    meters_per_second_t radialSpeed = (unitVecRadial.X().value() * chassisSpeed.vx) +
                                      (unitVecRadial.Y().value() * chassisSpeed.vy);

   //rotating radial vector to form tangential vector. using dot product to find the tangentialSpeed
   frc::Translation2d unitVecTangential = unitVecRadial.RotateBy(90_deg);
   units::meters_per_second_t tangentialSpeed = (unitVecTangential.X().value() * chassisSpeed.vx) +
                                                (unitVecTangential.Y().value() * chassisSpeed.vy);

   SpeedComponents speedComponents;
   speedComponents.radialSpeed = radialSpeed;
   speedComponents.tangentialSpeed = tangentialSpeed;

return speedComponents;
}