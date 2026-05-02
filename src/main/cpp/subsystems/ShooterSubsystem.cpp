#include <subsystems/ShooterSubsystem.h>

using namespace units::math;

ShooterSubsystem::ShooterSubsystem(DriveSubsystem& drive, FlyWheelSubsystem& flyWheel, HoodSubsystem& hood, FeederSubsystem& feeder, IntakeSubsystem& intake)
: m_flyWheel(flyWheel),
  m_hood(hood),
  m_feeder(feeder),
  m_intake (intake),
  m_drive(drive)
{

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

void ShooterSubsystem::Periodic(){
    units::meter_t distanceToTarget = m_drive.DistanceToTarget();

    units::turn_t targetAngle = (distanceToTarget > FlyWheelConstants::kRangeThreshold) ? FlyWheelConstants::kMinAngle : FlyWheelConstants::kMaxAngle;
    units::turns_per_second_t flywheelVelocity = m_flyWheel.CalculateFlyWheelSpeed(distanceToTarget, targetAngle);
    
    m_hood.SetTargetAngle(targetAngle);
    m_flyWheel.SetTargetVelocity(flywheelVelocity);
}

ShootSolution ShooterSubsystem::CompensateShootSolutionForRobotVelocity(ShootSolution ballSolution, meters_per_second_t robotRadialSpeed) {

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
