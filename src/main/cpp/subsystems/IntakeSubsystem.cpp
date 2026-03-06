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
    .SetIdleMode(rev::spark::SparkMaxConfig::kBrake);
    
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
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);

    ExtendRetractMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(IntakeConstants::extendRetract::kP)
      .I(IntakeConstants::extendRetract::kI)
      .D(IntakeConstants::extendRetract::kD)
      .OutputRange(-1, 1);
    
    m_extendRetractMotor.Configure(
      ExtendRetractMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    m_extendRetractEncoder.SetPosition(0);



    //follower motor config 
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

    ExtendRetractMotorConfig.softLimit
    .ForwardSoftLimit(IntakeConstants::kRetractSoftLimit.value()).ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(IntakeConstants::kExtendSoftLimit.value()).ReverseSoftLimitEnabled(true);

    ExtendRetractMotorConfig.encoder
    .PositionConversionFactor(IntakeConstants::kIntakeGearRatio)
    .VelocityConversionFactor(IntakeConstants::kIntakeGearRatio);

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