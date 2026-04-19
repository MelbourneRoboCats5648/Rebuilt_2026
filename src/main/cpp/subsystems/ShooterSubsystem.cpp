#include "subsystems/ShooterSubsystem.h"
#include "constants/FieldConstants.h"

#include <rev/config/SparkMaxConfig.h>

#include <units/math.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;

ShooterSubsystem::ShooterSubsystem(DriveSubsystem& drive)
: m_motor(HardwareConstants::kShooterFlywheelID, HardwareConstants::kPhoenixCAN),
  m_follower(HardwareConstants::kShooterFlywheelFollowerID, HardwareConstants::kPhoenixCAN),
  m_drive(drive)
  {
    TalonFXConfiguration flyWheelMotorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(flyWheelMotorConfig);
    m_follower.GetConfigurator().Apply(flyWheelMotorConfig);

    m_follower.SetControl(Follower{m_motor.GetDeviceID(), false});

    m_rotorVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/RotorVel").Publish();
    m_motorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/MotorWheelVel").Publish();
    m_followerMotorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/FollowerMotorWheelVel").Publish();
    m_motorCurrentPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/MotorCurrent").Publish();
    m_followerMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/FollowerMotorCurrent").Publish();


    m_shooterVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/Voltage").Publish();
    m_shooterTargetVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/TargetVelocity").Publish();

    m_adjustedSpeedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/adjustedSpeed").Publish();
    m_requiredSpedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/requiredSpeed").Publish();

    // uncomment below to allow smart dashboard to display 'ShooterVelocity' string
    // frc::SmartDashboard::PutNumber("ShooterVelocity", 0.0); // initialise shooter target velocity with default of 0.0
}

TalonFXConfiguration ShooterSubsystem::createMotorConfig(){
    TalonFXConfiguration motorConfig;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    motorConfig.Feedback.SensorToMechanismRatio = ShooterConstants::kgearRatio;
    motorConfig.Slot0.kP = ShooterConstants::motor::kP;
    motorConfig.Slot0.kI = ShooterConstants::motor::kI;
    motorConfig.Slot0.kD = ShooterConstants::motor::kD;
    motorConfig.Slot0.kV = ShooterConstants::motor::kV;
    motorConfig.Slot0.kS = ShooterConstants::motor::kS;
    motorConfig.Slot0.kA = ShooterConstants::motor::kA;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50_A; // fixme - these values will have to be changed
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60_A;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    motorConfig.MotorOutput.Inverted = true;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;

    return motorConfig;
}

void ShooterSubsystem::Periodic() {
        /* publish current state */

        // uncomment below to allow target velocity to be set via smart dashboard
        //m_targetVelocity = units::turns_per_second_t{frc::SmartDashboard::GetNumber("ShooterVelocity", 0.0)};

        units::meter_t distanceToTarget = m_drive.DistanceToTarget();
        units::turn_t targetAngle = (distanceToTarget > ShooterConstants::kRangeThreshold) ? ShooterConstants::kMinAngle : ShooterConstants::kMaxAngle;
        
        units::turns_per_second_t flywheelVelocity = CalculateFlyWheelSpeed(distanceToTarget, targetAngle);
        SetTargetVelocity(flywheelVelocity); 

        //SetFlywheelVelocityAndAngle(distanceToTarget);
        //SetTargetVelocity(ShooterConstants::kMaxAngularVelocity);

        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_follower.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
        m_shooterVoltagePub.Set(m_motor.GetMotorVoltage().GetValueAsDouble());

        m_requiredSpedPub.Set(m_requiredSpeed);
        m_adjustedSpeedPub.Set(m_adjustedSpeed);


}

void ShooterSubsystem::Shoot(units::volt_t volts){
    m_motor.SetVoltage(volts);
}

void ShooterSubsystem::ShootAngularVelocity(units::turns_per_second_t angularVelocity) {
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage(angularVelocity * m_scaleFlywheelVelocity);
    m_motor.SetControl(velocityVoltage);
    m_shooterTargetVelPub.Set(angularVelocity.value());
}

void ShooterSubsystem::SetTargetVelocity(units::turns_per_second_t velocity)
{
    m_targetVelocity = std::clamp(
        velocity, 
        ShooterConstants::kMinAngularVelocity, 
        ShooterConstants::kMaxAngularVelocity);

}

units::turns_per_second_t ShooterSubsystem::GetTargetVelocity() const{
    return m_targetVelocity;
}

frc2::CommandPtr ShooterSubsystem::ShootCommand() {
    return Run([this]{
                ShootAngularVelocity(m_targetVelocity);
            }).FinallyDo([this] { m_motor.StopMotor(); });
}

// This command will be used to set the velocity of the flywheel shooter along with the default shoot command
frc2::CommandPtr ShooterSubsystem::SetTargetVelocityCommand(units::turns_per_second_t angularVelocity) {
    return RunOnce([this, angularVelocity]{
                SetTargetVelocity(angularVelocity);
            });
}

frc2::CommandPtr ShooterSubsystem::IncreaseFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity += ShooterConstants::kFlywheelVelScalingIncrement;
        m_scaleFlywheelVelocity = std::clamp(m_scaleFlywheelVelocity, ShooterConstants::kMinFlywheelVelocityScaling, ShooterConstants::kMaxFlywheelVelocityScaling);
    });
}

frc2::CommandPtr ShooterSubsystem::DecreaseFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity -= ShooterConstants::kFlywheelVelScalingIncrement;
        m_scaleFlywheelVelocity = std::clamp(m_scaleFlywheelVelocity, ShooterConstants::kMinFlywheelVelocityScaling, ShooterConstants::kMaxFlywheelVelocityScaling);
    });
}

frc2::CommandPtr ShooterSubsystem::ResetFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity = ShooterConstants::kDefaultFlywheelVelocityScaling;
    });
}

/* old look up table
void ShooterSubsystem::SetFlywheelVelocityAndAngle(meter_t distanceToTarget)
{
    units::degree_t angle;
    units::turns_per_second_t velocity;

    if (distanceToTarget > 3.3_m)
    {
        angle = ShooterConstants::kMinAngle;
        velocity = 45_tps;
    }
    else if(distanceToTarget > 2.8_m)
    {
        angle = ShooterConstants::kMinAngle; // max angle could also work
        velocity = 50_tps;
    }
    else if (distanceToTarget > 2.5_m)
    {
        angle = ShooterConstants::kMaxAngle;
        velocity = 45_tps;
    }
    else if (distanceToTarget > 2.0_m)
    {
        angle = ShooterConstants::kMaxAngle;
        velocity = 43_tps;
    }
    else
    {
        angle = ShooterConstants::kMaxAngle;
        velocity = 40_tps;
    }

    SetTargetAngle(angle);
    SetTargetVelocity(velocity);
}

*/

units::turns_per_second_t ShooterSubsystem::CalculateFlyWheelSpeed(meter_t distance, degree_t angle) {
    meters_per_second_t requiredSpeed = CalculateBallSpeed(distance, angle);
    meters_per_second_t adjustedSpeed = AdjustedBallSpeed(requiredSpeed);

    m_requiredSpeed = requiredSpeed.value();
    m_adjustedSpeed = adjustedSpeed.value();

    double metresPerTurn = 2 * std::numbers::pi * ShooterConstants::kFlyWheelRadius.value();
    return units::turns_per_second_t{adjustedSpeed.value() / metresPerTurn};
}

// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 

meters_per_second_t ShooterSubsystem::CalculateBallSpeed(meter_t distance, degree_t angle) {
        auto cosine = cos(angle);
        auto tangent = tan(angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * pow<2>(distance)) /
                (2 * pow<2>(cosine) * (distance * tangent - adjustedHeight))
            );

        return speed;
}

meters_per_second_t ShooterSubsystem::AdjustedBallSpeed(meters_per_second_t actualSpeed) {
    // coefficients found from curve fitting
    double a = -0.8418;
    double b = 14.6320;
    double c = -46.5525;

    double x = actualSpeed.value();

    meters_per_second_t adjustedSpeed =  meters_per_second_t(a * x * x + b * x + c);
    return adjustedSpeed;
}

ShootSolution ShooterSubsystem::CompensateShootSolutionForRobotVelocity(degree_t ballAngle, meters_per_second_t ballSpeed, meters_per_second_t robotRadialSpeed) {

    meters_per_second_t horizontalBallSpeed =
        ballSpeed * units::math::cos(ballAngle);

    meters_per_second_t verticalBallSpeed = 
        ballSpeed * units::math::sin(ballAngle);

    meters_per_second_t compensatedHorizontalSpeed = 
        horizontalBallSpeed - robotRadialSpeed;

    meters_per_second_t compensatedVerticalSpeed = verticalBallSpeed;

    meters_per_second_t compensatedBallSpeed =
        sqrt(
            pow<2>(compensatedHorizontalSpeed) + pow<2>(compensatedVerticalSpeed)
            );

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
