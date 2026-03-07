#include <subsystems/IntakeSubsystem.h>

#include <constants/HardwareConstants.h>

#include <rev/config/SparkMaxConfig.h>

IntakeSubsystem::IntakeSubsystem() :
m_extendRetractMotor(HardwareConstants::kExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_followerExtendRetractMotor(HardwareConstants::kFollowerExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_intakeMotor(HardwareConstants::kIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless)

{
    //intake motor config
    rev::spark::SparkMaxConfig intakeMotorConfig;

    intakeMotorConfig
      .SmartCurrentLimit(IntakeConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
    
    intakeMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(IntakeConstants::intake::kP)
      .I(IntakeConstants::intake::kI)
      .D(IntakeConstants::intake::kD)
      .OutputRange(-1, 1);

    m_intakeMotor.Configure(
      intakeMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    //extend retract motor config
    rev::spark::SparkMaxConfig ExtendRetractMotorConfig;

    ExtendRetractMotorConfig
      .SmartCurrentLimit(IntakeConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
      .Inverted(true);

    ExtendRetractMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(IntakeConstants::extendRetract::kP)
      .I(IntakeConstants::extendRetract::kI)
      .D(IntakeConstants::extendRetract::kD)
      .OutputRange(-1, 1);

    ExtendRetractMotorConfig.softLimit
      .ForwardSoftLimit(IntakeConstants::kExtendSoftLimit.value()).ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(IntakeConstants::kRetractSoftLimit.value()).ReverseSoftLimitEnabled(true);

    const double metresPerTurn = IntakeConstants::kExtendRetractSprocketDia * std::numbers::pi * IntakeConstants::kExtendRetractGearRatio;

    ExtendRetractMotorConfig.encoder
      .PositionConversionFactor(metresPerTurn)
      .VelocityConversionFactor(metresPerTurn);
    
    m_extendRetractMotor.Configure(
      ExtendRetractMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    m_extendRetractEncoder.SetPosition(0);


    // follower extend/retract motor config 
    rev::spark::SparkMaxConfig FollowerExtendRetractMotorConfig;

    FollowerExtendRetractMotorConfig
    .SmartCurrentLimit(IntakeConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
    .Follow(m_extendRetractMotor, true);  // invert = true

    m_followerExtendRetractMotor.Configure(
      FollowerExtendRetractMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    ConfigurePublishers();
}

void IntakeSubsystem::ConfigurePublishers()
{
    m_extendRetractPositionPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetractPosition").Publish();
    m_extendRetractMotorCurrentPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetractMotorCurrent").Publish();
    m_followerExtendRetractMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/FollowerExtendRetractMotorCurrent").Publish();
}

void IntakeSubsystem::Periodic()
{
    /* publish current state */
    m_extendRetractPositionPub.Set(m_extendRetractEncoder.GetPosition());
    m_extendRetractMotorCurrentPub.Set(m_extendRetractMotor.GetOutputCurrent());
    m_followerExtendRetractMotorCurrentPub.Set(m_followerExtendRetractMotor.GetOutputCurrent());
}

turns_per_second_t IntakeSubsystem::CalculateIntakeSpeed(meters_per_second_t forwardRobotSpeed)
{
    // calculating the required speed of the wheel surface (tangent) with respect to the ground
    meters_per_second_t adjustedIntakeSurfaceSpeed = IntakeConstants::kTargetIntakeSurfaceSpeed + forwardRobotSpeed;

    // calculate angular velocity of intake wheel from the desired surface tangential speed
    meter_t distancePerTurn = IntakeConstants::PI * IntakeConstants::kIntakeWheelDiameter;

    turns_per_second_t requiredIntakeWheelSpeed = turns_per_second_t{adjustedIntakeSurfaceSpeed.value() / distancePerTurn.value()};

    return requiredIntakeWheelSpeed;
}

void IntakeSubsystem::GoToPosition(units::meter_t position){
  m_extendRetractController.SetReference(position.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::SetIntakeVoltage(units::volt_t voltage)
{
  m_intakeMotor.SetVoltage(voltage);
}