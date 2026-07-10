#include <subsystems/ShooterSubsystem.h>
#include <units/math.h>

using namespace units::math;

ShooterSubsystem::ShooterSubsystem(DriveSubsystem& drive, IntakeSubsystem& intake)
: m_intake (intake),
  m_drive(drive)
{
    m_staticAnglePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/Static/Angle").Publish(); // degree
    m_staticVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/Static/BallVelocity").Publish(); // m/s
        
    m_radialCompensatedAnglePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/RadialCompensated/Angle").Publish(); // degree
    m_radialCompensatedVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/RadialCompensated/BallVelocity").Publish(); // m/s
    m_radialCompensatedAngleDeltaPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/RadialCompensated/DeltaAngle").Publish(); // degree
    m_radialCompensatedVelocityDeltaPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/RadialCompensated/DeltaBallVelocity").Publish(); // m/s

    m_tangentialCompensatedAngleDeltaPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/TangentialCompensated/DeltaAngle").Publish(); // degree
    m_tangentialCompensatedVelocityDeltaPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/TangentialCompensated/DeltaBallVelocity").Publish(); // m/s

    m_sotmAnglePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/Angle").Publish(); // degree
    m_sotmVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/BallVelocity").Publish(); // m/s
    m_sotmYawPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/Yaw").Publish(); // degree

    m_flywheelVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/FlywheelVelocity").Publish(); // turns/sec
}

frc2::CommandPtr ShooterSubsystem::ShootCommand() {
    return frc2::cmd::Parallel(
        m_flyWheel.SpinFlyWheelCommand(),
        frc2::cmd::Wait(FlyWheelConstants::kRampTime).AndThen(m_feeder.FeedCommand())
    );
}

frc2::CommandPtr ShooterSubsystem::ShootCommandWithHood() {
    return frc2::cmd::Parallel(
        m_flyWheel.SpinFlyWheelCommand(),
        m_hood.GoToAngleCommand(),
        frc2::cmd::Wait(FlyWheelConstants::kRampTime).AndThen(m_feeder.FeedCommand())
    );
}

frc2::CommandPtr ShooterSubsystem::ShootCommandWithFeeder(units::second_t feedTime) {
    return ShootCommand().WithTimeout(FlyWheelConstants::kRampTime + feedTime);
}

// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 
meters_per_second_t ShooterSubsystem::CalculateRequiredBallSpeed(meter_t distance, degree_t angle) {
        auto cosine = units::math::cos(angle);
        auto tangent = units::math::tan(angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - HoodConstants::hoodHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * units::math::pow<2>(distance)) /
                (2 * units::math::pow<2>(cosine) * (distance * tangent - adjustedHeight))
            );

        return speed;
}

void ShooterSubsystem::Periodic(){

    units::meter_t distanceToTarget = m_drive.DistanceToTarget();
    units::turn_t targetAngle = GetBestAngleForDistance(distanceToTarget);
    units::meters_per_second_t ballSpeed = CalculateRequiredBallSpeed(distanceToTarget, targetAngle);
    
    // the static shoot solution (before compensation for robot movement)
    ShootSolution staticShootSolution;
    staticShootSolution.angle = targetAngle;
    staticShootSolution.speed = ballSpeed;
    m_staticAnglePub.Set(staticShootSolution.angle.value());
    m_staticVelocityPub.Set(staticShootSolution.speed.value());

    // finding robot's radial and tangential speeds
    SpeedComponents speedComponents = m_drive.GetSpeedComponents();

    // take static shoot solution and provide a radial speed compensated shoot solution
    ShootSolution radialShootSolution = CompensateForRadialSpeed(staticShootSolution, speedComponents.radialSpeed);
    m_radialCompensatedAnglePub.Set(radialShootSolution.angle.value());
    m_radialCompensatedVelocityPub.Set(radialShootSolution.speed.value());
    m_radialCompensatedAngleDeltaPub.Set((radialShootSolution.angle - targetAngle).value());
    m_radialCompensatedVelocityDeltaPub.Set((radialShootSolution.speed - ballSpeed).value());

    // take radial speed compensated shoot solution and add tangential speed compensation for the final SOTM solution
    // Also provide compensated yaw angle for drive train
    ShootOnTheMoveSolution movingShootSolution = CompensateYawForTangentialSpeed(radialShootSolution, speedComponents.tangentialSpeed);
    m_sotmAnglePub.Set(movingShootSolution.shootSolution.angle.value());
    m_sotmVelocityPub.Set(movingShootSolution.shootSolution.speed.value());
    m_sotmYawPub.Set(movingShootSolution.yawAngle.value());

    m_tangentialCompensatedAngleDeltaPub.Set((movingShootSolution.shootSolution.angle - radialShootSolution.angle).value());
    m_tangentialCompensatedVelocityDeltaPub.Set((movingShootSolution.shootSolution.speed - radialShootSolution.speed).value());
    
    units::turns_per_second_t flywheelVelocity = m_flyWheel.CalculateFlyWheelSpeed(movingShootSolution.shootSolution.speed);
    
    units::degree_t compensatedHoodAngle = movingShootSolution.shootSolution.angle;
    units::degree_t compensatedYawAngle = movingShootSolution.yawAngle;

    m_flywheelVelocityPub.Set(flywheelVelocity.value());

    m_hood.SetTargetAngle(AdjustAngle(compensatedHoodAngle));
    m_flyWheel.SetTargetVelocity(flywheelVelocity);
    m_drive.SetYawAngle(compensatedYawAngle);    
}

// this compensates the required hood angle based on tuning 
units::degree_t ShooterSubsystem::AdjustAngle(units::degree_t angle){
    double calculatedAngle = angle.value() * HoodConstants::kAngleScalingFactor + HoodConstants::kScalingFactorConstant;
    return degree_t(calculatedAngle);
}

ShootSolution ShooterSubsystem::CompensateForRadialSpeed(ShootSolution ballSolution, meters_per_second_t robotRadialSpeed) {

    meters_per_second_t horizontalBallSpeed =
        ballSolution.speed * units::math::cos(ballSolution.angle);

    meters_per_second_t verticalBallSpeed = 
        ballSolution.speed * units::math::sin(ballSolution.angle);

    meters_per_second_t compensatedHorizontalSpeed = 
        horizontalBallSpeed - robotRadialSpeed;

    meters_per_second_t compensatedVerticalSpeed = verticalBallSpeed;

    meters_per_second_t compensatedBallSpeed = units::math::hypot(compensatedHorizontalSpeed, compensatedVerticalSpeed);

    degree_t compensatedBallAngle =
        units::math::atan2( 
            compensatedVerticalSpeed, compensatedHorizontalSpeed
        );

    ShootSolution solution;
    solution.angle = compensatedBallAngle;
    solution.speed = compensatedBallSpeed; 

    return solution;
}

ShootOnTheMoveSolution ShooterSubsystem::CompensateYawForTangentialSpeed(ShootSolution solution, units::meters_per_second_t robotTangentialSpeed) {
    
    units::meters_per_second_t requiredBallShootingSpeed = solution.speed;
    units::degree_t requiredHoodAngle = solution.angle;
    
    units::meters_per_second_t horizontalRadialBallSpeed = requiredBallShootingSpeed * units::math::cos(requiredHoodAngle);
    meters_per_second_t verticalBallSpeed = requiredBallShootingSpeed * units::math::sin(solution.angle);

    // the horizontal component is the projection of the compensated ball speed vector onto the horizontal plane
    meters_per_second_t horizontalComponent = units::math::hypot(horizontalRadialBallSpeed, robotTangentialSpeed);

    meters_per_second_t compensatedSpeed = units::math::hypot(horizontalComponent, verticalBallSpeed);
    degree_t compensatedAngle = units::math::atan2(verticalBallSpeed, horizontalComponent);

    ShootSolution shootSolution;
    shootSolution.angle = compensatedAngle;
    shootSolution.speed = compensatedSpeed;

    units::degree_t ballYawAngle = units::math::atan2(robotTangentialSpeed, horizontalRadialBallSpeed);
    units::degree_t compensatedYawAngle = -ballYawAngle;

    ShootOnTheMoveSolution movingSolution;
    movingSolution.shootSolution = shootSolution;
    movingSolution.yawAngle = compensatedYawAngle;

    return movingSolution;
}

units::degree_t ShooterSubsystem::GetBestAngleForDistance(meter_t distanceToTarget){
    auto gradient = ((HoodConstants::kMidAngle - HoodConstants::kMaxAngle) 
                        / ((HoodConstants::kUpperRangeThreshold - HoodConstants::kLowerRangeThreshold)));

    units::degree_t angle = (distanceToTarget - HoodConstants::kLowerRangeThreshold) * gradient + HoodConstants::kMaxAngle;

    return std::clamp(angle, HoodConstants::kMidAngle, HoodConstants::kMaxAngle);
}


frc2::CommandPtr ShooterSubsystem::SetFlywheelVelocityCommand(units::turns_per_second_t angularVelocity)
{
    return m_flyWheel.SetTargetVelocityCommand(angularVelocity);
}

frc2::CommandPtr ShooterSubsystem::RetractHoodToLimitCommand() {
    return m_hood.RetractToLimitCommand();
}

frc2::CommandPtr ShooterSubsystem::SetHoodTargetAngleCommand(units::degree_t angle) {
    return m_hood.SetTargetAngleCommand(angle);
}

bool ShooterSubsystem::IsStalling() {
    return m_feeder.IsStalling();
}

frc2::CommandPtr ShooterSubsystem::ReverseFeedCommand(){
    return m_feeder.ReverseFeedCommand();
}