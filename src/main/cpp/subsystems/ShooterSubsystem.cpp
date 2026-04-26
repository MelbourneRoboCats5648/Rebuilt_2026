#include <subsystems/ShooterSubsystem.h>

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
        m_flyWheel.ShootCommand(),
        frc2::cmd::Wait(FlyWheelConstants::kRampTime).AndThen(m_feeder.FeedCommand())
    );
}

frc2::CommandPtr ShooterSubsystem::ShootCommandWithHood() {
    return frc2::cmd::Parallel(
        m_flyWheel.ShootCommand(),
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
}
