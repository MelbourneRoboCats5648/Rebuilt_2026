#include <subsystems/ShooterSubsystem.h>

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
        .GetDoubleTopic("Shooter/RadialCompensated/Angle").Publish(); // degree
    m_radialCompensatedVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/RadialCompensated/BallVelocity").Publish(); // m/s

    m_moveAnglePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/Angle").Publish(); // degree
    m_moveVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/BallVelocity").Publish(); // m/s
    m_moveYawPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/Yaw").Publish(); // degree
        
    m_moveCompensatedAnglePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/CompensatedAngle").Publish(); // degree
    m_moveFlywheelVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/FlywheelVelocity").Publish(); // turns/sec
    m_moveCompensatedYawPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/ShootOnTheMove/CompensatedYaw").Publish(); // degree
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

meters_per_second_t ShooterSubsystem::CalculateRequiredBallSpeed(meter_t distance, degree_t angle) {
        auto cosine = cos(angle);
        auto tangent = tan(angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - FlyWheelConstants::startHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * pow<2>(distance)) /
                (2 * pow<2>(cosine) * (distance * tangent - adjustedHeight))
            );

        return speed;
}

void ShooterSubsystem::Periodic(){
    units::meter_t distanceToTarget = m_drive.DistanceToTarget();

    units::turn_t targetAngle = (distanceToTarget > FlyWheelConstants::kRangeThreshold) ? HoodConstants::kMinAngle : HoodConstants::kMaxAngle;
    // fixme(MRT) - could use a lookup table like in SCR instead of simple implementation above
    //              targetAngle = GetBestAngleForDistance(distanceToTarget);

    units::meters_per_second_t ballSpeed = CalculateRequiredBallSpeed(distanceToTarget, targetAngle);
    
    // the static shoot solution (before compensation for robot movement)
    ShootSolution shootSolution;
    shootSolution.angle = targetAngle;
    shootSolution.speed = ballSpeed;
    m_staticAnglePub.Set(shootSolution.angle.value());
    m_staticVelocityPub.Set(shootSolution.speed.value());

    // finding robot's radial and tangential speeds
    SpeedComponents speedComponents = m_drive.GetSpeedComponents();

    shootSolution = CompensateForRadialSpeed(shootSolution, speedComponents.radialSpeed);
    m_radialCompensatedAnglePub.Set(shootSolution.angle.value());
    m_radialCompensatedVelocityPub.Set(shootSolution.speed.value());

    ShootOnTheMoveSolution movingShootSolution = CompensateYawForTangentialSpeed(shootSolution, speedComponents.tangentialSpeed);
    m_moveAnglePub.Set(movingShootSolution.shootSolution.angle.value());
    m_moveVelocityPub.Set(movingShootSolution.shootSolution.speed.value());
    m_moveYawPub.Set(movingShootSolution.yawAngle.value());
    
    units::turns_per_second_t flywheelVelocity = m_flyWheel.CalculateFlyWheelSpeed(movingShootSolution.shootSolution.speed);
    units::degree_t compensatedAngle = movingShootSolution.shootSolution.angle;
    units::degree_t compensatedYawAngle = movingShootSolution.yawAngle;
    m_moveCompensatedAnglePub.Set(compensatedAngle.value());
    m_moveFlywheelVelocityPub.Set(flywheelVelocity.value());
    m_moveCompensatedYawPub.Set(compensatedYawAngle.value());
    
    // fixme(MRT) - uncomment below after testing
    //m_hood.SetTargetAngle(compensatedAngle);
    //m_flyWheel.SetTargetVelocity(flywheelVelocity);
    m_drive.SetYawAngle(compensatedYawAngle);
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
        atan2( 
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
    
    units::meters_per_second_t horizontalRadialBallSpeed = requiredBallShootingSpeed * cos(requiredHoodAngle);
    
    units::degree_t ballYawAngle = units::degree_t(atan2(robotTangentialSpeed.value(), horizontalRadialBallSpeed.value()));
    units::degree_t compensatedYawAngle = -ballYawAngle;

    meters_per_second_t verticalBallSpeed = horizontalRadialBallSpeed * sin(solution.angle);

    // the horizontal component is the projection of the compensated ball vector onto the horizontal plane
    meters_per_second_t horizontalComponent = units::math::hypot(horizontalRadialBallSpeed, robotTangentialSpeed);

    meters_per_second_t compensatedSpeed = units::math::hypot(horizontalComponent, verticalBallSpeed);
    degree_t compensatedAngle = degree_t(atan2(verticalBallSpeed.value(), horizontalComponent.value()));

    ShootSolution shootSolution;
    shootSolution.angle = compensatedAngle;
    shootSolution.speed = compensatedSpeed;

    ShootOnTheMoveSolution movingSolution;
    movingSolution.shootSolution = shootSolution;
    movingSolution.yawAngle = compensatedYawAngle;

    return movingSolution;
}

/* Similar to old look up table from SCR to choose best angle for given distance break points (need to find this from tuning)
units::degree_t angle ShooterSubsystem::GetBestAngleForDistance(meter_t distanceToTarget)
{
    units::degree_t angle;

    if (distanceToTarget > 3.3_m)
    {
        angle = ShooterConstants::kMinAngle;
    }
    else if(distanceToTarget > 2.8_m)
    {
        angle = ShooterConstants::kMinAngle; // max angle could also work
    }
    else if (distanceToTarget > 2.5_m)
    {
        angle = ShooterConstants::kMaxAngle;
    }
    else if (distanceToTarget > 2.0_m)
    {
        angle = ShooterConstants::kMaxAngle;
    }
    else
    {
        angle = ShooterConstants::kMaxAngle;
    }

    return angle;
}
*/

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

frc2::CommandPtr ShooterSubsystem::IncreaseFeederVoltageDifference() {
    return m_feeder.IncreaseFeederVoltageDifference();
}

frc2::CommandPtr ShooterSubsystem::DecreaseFeederVoltageDifference() {
    return m_feeder.DecreaseFeederVoltageDifference();
}