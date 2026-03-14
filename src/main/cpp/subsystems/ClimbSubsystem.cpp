#include "subsystems/ClimbSubsystem.h"

#include <rev/config/SparkMaxConfig.h>

ClimbSubsystem::ClimbSubsystem()
: 
  m_motor(HardwareConstants::kClimbMotorID, rev::spark::SparkMax::MotorType::kBrushless),
  m_followerMotor(HardwareConstants::kClimbFollowerMotorID, rev::spark::SparkMax::MotorType::kBrushless)
{
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
    .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kBrake);

    //motorConfig.softLimit
    //.ForwardSoftLimit(ClimbConstants::kExtendSoftLimit.value()).ForwardSoftLimitEnabled(true)
    //.ReverseSoftLimit(ClimbConstants::kRetractSoftLimit.value()).ReverseSoftLimitEnabled(true);

    const double metresPerTurn = ClimbConstants::kClimbSprocketDia * std::numbers::pi * ClimbConstants::kGearRatio;

    motorConfig.encoder
      .PositionConversionFactor(metresPerTurn)
      .VelocityConversionFactor(metresPerTurn / 60); // by default Spark Max returns RPM; we want to convert to m/s here (hence we divide by 60 too)

    motorConfig.encoder
    .PositionConversionFactor(ClimbConstants::kGearRatio)
    .VelocityConversionFactor(ClimbConstants::kGearRatio);
    
    m_motor.Configure(
      motorConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );


    rev::spark::SparkMaxConfig followerConfig;

    followerConfig
      .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kBrake)
      .Follow(m_motor, true);  // invert = true

    m_followerMotor.Configure(
      followerConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );


};

frc2::CommandPtr ClimbSubsystem::ClimbUpCommand() {
  return Run([this] { m_motor.Set(ClimbConstants::kMaxVoltage.value()); });
}

// frc2::CommandPtr ClimbSubsystem::ClimbDownCommand() {
//   return Run([this] { m_motor.Set(-0.1); });
// }



