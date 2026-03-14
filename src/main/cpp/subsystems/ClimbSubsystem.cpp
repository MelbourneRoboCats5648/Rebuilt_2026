#include "subsystems/ClimbSubsystem.h"

#include <rev/config/SparkMaxConfig.h>
#include <networktables/NetworkTableInstance.h>

ClimbSubsystem::ClimbSubsystem()
: 
  m_motor(HardwareConstants::kClimbMotorID, rev::spark::SparkMax::MotorType::kBrushless),
  m_followerMotor(HardwareConstants::kClimbFollowerMotorID, rev::spark::SparkMax::MotorType::kBrushless)
{
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
    .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kBrake)
    .Inverted(true);

    motorConfig.softLimit
      .ForwardSoftLimit(ClimbConstants::kExtendSoftLimit.value()).ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(ClimbConstants::kRetractSoftLimit.value()).ReverseSoftLimitEnabled(true);

    const double metresPerTurn = ClimbConstants::kClimbSprocketDia * std::numbers::pi * ClimbConstants::kGearRatio;
    // fixme

    motorConfig.encoder
      .PositionConversionFactor(metresPerTurn)
      .VelocityConversionFactor(metresPerTurn);
    
    m_motor.Configure(
      motorConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );

    rev::spark::SparkMaxConfig followerConfig;

    followerConfig
      .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kBrake); // less risk than follow since it didn't work with feeder

    m_followerMotor.Configure(
      followerConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );

    m_positionPub = nt::NetworkTableInstance::GetDefault()
      .GetDoubleTopic("Climb/Position").Publish();
    m_currentPub = nt::NetworkTableInstance::GetDefault()
      .GetDoubleTopic("Climb/Current").Publish();
    m_followerCurrentPub = nt::NetworkTableInstance::GetDefault()
      .GetDoubleTopic("Climb/FollowerCurrent").Publish();
};

void ClimbSubsystem::Periodic() {
  m_positionPub.Set(m_motor.GetEncoder().GetPosition());
  m_currentPub.Set(m_motor.GetOutputCurrent());
  m_followerCurrentPub.Set(m_followerMotor.GetOutputCurrent());
}

frc2::CommandPtr ClimbSubsystem::RetractCommand() {
  return Run([this] {
    m_motor.SetVoltage(-ClimbConstants::kRetractVoltage);
    m_followerMotor.SetVoltage(-ClimbConstants::kRetractVoltage);
  }).FinallyDo([this] {
    m_motor.StopMotor();
    m_followerMotor.StopMotor();
  });
}

frc2::CommandPtr ClimbSubsystem::ExtendCommand() {
  return Run([this] {
    m_motor.SetVoltage(ClimbConstants::kExtendVoltage);
    m_followerMotor.SetVoltage(ClimbConstants::kExtendVoltage);
  }).FinallyDo([this] {
    m_motor.StopMotor();
    m_followerMotor.StopMotor();
  });
}





