#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"

#include <units/math.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;

ShooterSubsystem::ShooterSubsystem()
: m_motor(HardwareConstants::kShooterMotorID, "rio"),
  m_follower(HardwareConstants::kShooterFollowerMotorID, "rio")
{
    TalonFXConfiguration motorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(motorConfig);
    m_follower.GetConfigurator().Apply(motorConfig);

    m_follower.SetControl(Follower{m_motor.GetDeviceID(), false});

    SetDefaultCommand(ShootCommand(0_tps));

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
};

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
};

void ShooterSubsystem::Periodic() {
        /* publish current state */
        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
}

void ShooterSubsystem::Shoot(units::volt_t volts){
    m_motor.SetVoltage(volts);
}

void ShooterSubsystem::ShootAngularVelocity(units::turns_per_second_t angularVelocity) {
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage(angularVelocity);
    m_motor.SetControl(velocityVoltage);
}

frc2::CommandPtr ShooterSubsystem::ShootCommand(units::turns_per_second_t angularVelocity) {
    return Run([this, angularVelocity]{
                ShootAngularVelocity(angularVelocity);
            });
};

units::turns_per_second_t ShooterSubsystem::CalculateFlyWheelSpeed(meter_t distance, degree_t angle) {
    meters_per_second_t ballSpeed = CalculateBallSpeed(distance, angle);

    double metresPerTurn = 2 * std::numbers::pi * ShooterConstants::kFlyWheelRadius.value();
    return units::turns_per_second_t{ballSpeed.value()/metresPerTurn};
};

// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 

meters_per_second_t ShooterSubsystem::CalculateBallSpeed(meter_t distance, degree_t angle) {
        auto cosine = cos(ShooterConstants::angle);
        auto tangent = tan(ShooterConstants::angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * pow<2>(ShooterConstants::distance)) /
                (2 * pow<2>(cosine) * (ShooterConstants::distance * tangent - adjustedHeight))
            );

        return speed;
};

meter_t ShooterSubsystem::CalculateDistanceBetweenPoints(frc::Translation2d p1, frc::Translation2d p2) {
    return p1.Distance(p2);
};