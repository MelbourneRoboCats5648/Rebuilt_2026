#include "subsystems/ClimbSubsystem.h"

#include <rev/config/SparkMaxConfig.h>

ClimbSubsystem::ClimbSubsystem(int motorCanID, int followerMotorCanID)
: m_motor(motorCanID, rev::spark::SparkMax::MotorType::kBrushless),
  m_followerMotor(followerMotorCanID, rev::spark::SparkMax::MotorType::kBrushless)
{
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
    .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);

    motorConfig.softLimit
    .ForwardSoftLimit(ClimbConstants::kExtendSoftLimit.value()).ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(ClimbConstants::kRetractSoftLimit.value()).ReverseSoftLimitEnabled(true);

    motorConfig.encoder
    .PositionConversionFactor(ClimbConstants::kGearRatio)
    .VelocityConversionFactor(ClimbConstants::kGearRatio);
    
    m_motor.Configure(
      motorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );


    rev::spark::SparkMaxConfig followerConfig;

    followerConfig
      .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
      .Follow(m_motor, true);  //

    m_followerMotor.Configure(
      followerConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

};



