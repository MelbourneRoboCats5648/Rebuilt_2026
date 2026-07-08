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
        .GetDoubleTopic("Flywheel/RotorVel").Publish();

    m_motorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/MotorWheelVel").Publish();
    m_followerMotorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/FollowerMotorWheelVel").Publish();
    m_motorCurrentPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/MotorCurrent").Publish();
    m_followerMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/FollowerMotorCurrent").Publish();

    m_flyWheelVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/Voltage").Publish();
    m_flyWheelTargetVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/TargetVelocity").Publish();

    m_adjustedSpeedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/adjustedBallSpeed").Publish();
    m_requiredSpeedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Flywheel/requiredBallSpeed").Publish();

    // uncomment below to allow smart dashboard to display 'ShooterVelocity' string
    //frc::SmartDashboard::PutNumber("ShooterVelocity", 42.0); // initialise shooter target velocity with default of 0.0
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
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60_A;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60_A;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    motorConfig.MotorOutput.Inverted = true;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;

    return motorConfig;
}

void FlyWheelSubsystem::Periodic() {
        /* publish current state */

        //uncomment below to allow target velocity to be set via smart dashboard
        //m_targetVelocity = units::turns_per_second_t{frc::SmartDashboard::GetNumber("ShooterVelocity", 0.0)};

        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_follower.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
        m_flyWheelVoltagePub.Set(m_motor.GetMotorVoltage().GetValueAsDouble());
}

void FlyWheelSubsystem::SpinFlyWheelVoltage(units::volt_t volts){
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

units::turns_per_second_t FlyWheelSubsystem::CalculateFlyWheelSpeed(meters_per_second_t ballSpeed) {
    meters_per_second_t adjustedSpeed = AdjustedBallSpeed(ballSpeed);

    m_requiredSpeedPub.Set(ballSpeed.value());
    m_adjustedSpeedPub.Set(adjustedSpeed.value());

    // circumferance
    units::meter_t metresPerTurn = 2 * std::numbers::pi * FlyWheelConstants::kFlyWheelRadius;
    units::frequency::hertz_t flywheelAngularRate = adjustedSpeed / metresPerTurn;
    units::turns_per_second_t flywheelVelocity = units::turns_per_second_t{flywheelAngularRate.value()};

    return flywheelVelocity;
}

meters_per_second_t FlyWheelSubsystem::AdjustedBallSpeed(meters_per_second_t actualSpeed) {
    double b = 2.3046;
    meters_per_second_t adjustedSpeed = (b * actualSpeed);

    return adjustedSpeed;
}