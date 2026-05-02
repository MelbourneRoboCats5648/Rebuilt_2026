#include "subsystems/FlyWheelSubsystem.h"
#include "constants/FieldConstants.h"

#include <rev/config/SparkMaxConfig.h>

#include <units/math.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;

FlyWheelSubsystem::FlyWheelSubsystem()
: m_motor(HardwareConstants::kShooterFlywheelID, HardwareConstants::kPhoenixCAN),
  m_follower(HardwareConstants::kShooterFlywheelFollowerID, HardwareConstants::kPhoenixCAN)
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


    m_flyWheelVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/Voltage").Publish();
    m_flyWheelTargetVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/TargetVelocity").Publish();

    m_adjustedSpeedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/adjustedSpeed").Publish();
    m_requiredSpedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/requiredSpeed").Publish();

    // uncomment below to allow smart dashboard to display 'ShooterVelocity' string
    // frc::SmartDashboard::PutNumber("ShooterVelocity", 0.0); // initialise shooter target velocity with default of 0.0
}

TalonFXConfiguration FlyWheelSubsystem::createMotorConfig(){
    TalonFXConfiguration motorConfig;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    motorConfig.Feedback.SensorToMechanismRatio = FlyWheelConstants::kgearRatio;
    motorConfig.Slot0.kP = FlyWheelConstants::motor::kP;
    motorConfig.Slot0.kI = FlyWheelConstants::motor::kI;
    motorConfig.Slot0.kD = FlyWheelConstants::motor::kD;
    motorConfig.Slot0.kV = FlyWheelConstants::motor::kV;
    motorConfig.Slot0.kS = FlyWheelConstants::motor::kS;
    motorConfig.Slot0.kA = FlyWheelConstants::motor::kA;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50_A; // fixme - these values will have to be changed
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60_A;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    motorConfig.MotorOutput.Inverted = true;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;

    return motorConfig;
}

void FlyWheelSubsystem::Periodic() {
        /* publish current state */

        // uncomment below to allow target velocity to be set via smart dashboard
        //m_targetVelocity = units::turns_per_second_t{frc::SmartDashboard::GetNumber("ShooterVelocity", 0.0)};

        //SetFlywheelVelocityAndAngle(distanceToTarget);
        //SetTargetVelocity(ShooterConstants::kMaxAngularVelocity);

        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_follower.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
        m_flyWheelVoltagePub.Set(m_motor.GetMotorVoltage().GetValueAsDouble());

        m_requiredSpedPub.Set(m_requiredSpeed);
        m_adjustedSpeedPub.Set(m_adjustedSpeed);


}

void FlyWheelSubsystem::SpinFlyWheel(units::volt_t volts){
    m_motor.SetVoltage(volts);
}

void FlyWheelSubsystem::SpinAtAngularVelocity(units::turns_per_second_t angularVelocity) {
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage(angularVelocity * m_scaleFlywheelVelocity);
    m_motor.SetControl(velocityVoltage);
    m_flyWheelTargetVelPub.Set(angularVelocity.value());
}

void FlyWheelSubsystem::SetTargetVelocity(units::turns_per_second_t velocity)
{
    m_targetVelocity = std::clamp(
        velocity, 
        FlyWheelConstants::kMinAngularVelocity, 
        FlyWheelConstants::kMaxAngularVelocity);

}

units::turns_per_second_t FlyWheelSubsystem::GetTargetVelocity() const{
    return m_targetVelocity;
}

frc2::CommandPtr FlyWheelSubsystem::SpinFlyWheelCommand() {
    return Run([this]{
                SpinAtAngularVelocity(m_targetVelocity);
            }).FinallyDo([this] { m_motor.StopMotor(); });
}

// This command will be used to set the velocity of the flywheel shooter along with the default shoot command
frc2::CommandPtr FlyWheelSubsystem::SetTargetVelocityCommand(units::turns_per_second_t angularVelocity) {
    return RunOnce([this, angularVelocity]{
                SetTargetVelocity(angularVelocity);
            });
}

frc2::CommandPtr FlyWheelSubsystem::IncreaseFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity += FlyWheelConstants::kFlywheelVelScalingIncrement;
        m_scaleFlywheelVelocity = std::clamp(m_scaleFlywheelVelocity, FlyWheelConstants::kMinFlywheelVelocityScaling, FlyWheelConstants::kMaxFlywheelVelocityScaling);
    });
}

frc2::CommandPtr FlyWheelSubsystem::DecreaseFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity -= FlyWheelConstants::kFlywheelVelScalingIncrement;
        m_scaleFlywheelVelocity = std::clamp(m_scaleFlywheelVelocity, FlyWheelConstants::kMinFlywheelVelocityScaling, FlyWheelConstants::kMaxFlywheelVelocityScaling);
    });
}

frc2::CommandPtr FlyWheelSubsystem::ResetFlywheelVelocity()
{
    return RunOnce([this] {
        m_scaleFlywheelVelocity = FlyWheelConstants::kDefaultFlywheelVelocityScaling;
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

units::turns_per_second_t FlyWheelSubsystem::CalculateFlyWheelSpeed(meter_t distance, degree_t angle) {
    meters_per_second_t requiredSpeed = CalculateBallSpeed(distance, angle);
    meters_per_second_t adjustedSpeed = AdjustedBallSpeed(requiredSpeed);

    m_requiredSpeed = requiredSpeed.value();
    m_adjustedSpeed = adjustedSpeed.value();

    double metresPerTurn = 2 * std::numbers::pi * FlyWheelConstants::kFlyWheelRadius.value();
    return units::turns_per_second_t{adjustedSpeed.value() / metresPerTurn};
}

// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 

meters_per_second_t FlyWheelSubsystem::CalculateBallSpeed(meter_t distance, degree_t angle) {
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

meters_per_second_t FlyWheelSubsystem::AdjustedBallSpeed(meters_per_second_t actualSpeed) {
    // coefficients found from curve fitting
    double a = -0.8418;
    double b = 14.6320;
    double c = -46.5525;

    double x = actualSpeed.value();

    meters_per_second_t adjustedSpeed =  meters_per_second_t(a * x * x + b * x + c);
    return adjustedSpeed;
}