#include "subsystems/ClimbSubsystem.h"

#include <rev/config/SparkMaxConfig.h>

ClimbSubsystem::ClimbSubsystem(int canID)
: m_motor(canID, rev::spark::SparkMax::MotorType::kBrushless)
{
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
      .SmartCurrentLimit(ClimbConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);

//this doesn't build when not commented out 
    //motorConfig.softLimit
    //.ForwardSoftLimit(maxLimit.value()).ForwardSoftLimitEnabled(true)
    //.ReverseSoftLimit(minLimit.value()).ReverseSoftLimitEnabled(true);

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
}