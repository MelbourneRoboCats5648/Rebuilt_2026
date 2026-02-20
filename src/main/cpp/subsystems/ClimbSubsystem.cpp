#include "subsystems/ClimbSubsystem.h"

#include <rev/config/SparkMaxConfig.h>

ClimbSubsystem::ClimbSubsystem(int motorCanID, int followerMotorCanID, 
  PIDConstants pidConst, 
  frc::TrapezoidProfile<units::meter>::Constraints pidProfile)
: 
  m_motor(motorCanID, rev::spark::SparkMax::MotorType::kBrushless),
  m_followerMotor(followerMotorCanID, rev::spark::SparkMax::MotorType::kBrushless),
  m_controller(pidConst.kP, pidConst.kI, pidConst.kD, pidProfile, ClimbConstants::kDt)

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
      .Follow(m_motor, true);  // invert = true

    m_followerMotor.Configure(
      followerConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    m_controller.SetTolerance(ClimbConstants::kElevatorPositionTolerance, ClimbConstants::kElevatorVelocityTolerance);


};

frc2::CommandPtr ClimbSubsystem::ClimbUpCommand() {
  return Run([this] { m_motor.Set(0.1); });
}

frc2::CommandPtr ClimbSubsystem::ClimbDownCommand() {
  return Run([this] { m_motor.Set(-0.1); });
}



