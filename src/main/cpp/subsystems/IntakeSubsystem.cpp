#include <subsystems/IntakeSubsystem.h>
#include <constants/HardwareConstants.h>

#include <rev/config/SparkMaxConfig.h>

IntakeSubsystem::IntakeSubsystem() :
m_ExtendRetractMotor(HardwareConstants::kExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
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

    m_intakeEncoder.SetPosition(0);



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
    
    m_ExtendRetractMotor.Configure(
      ExtendRetractMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );


    //follower motor config
    rev::spark::SparkMaxConfig FollowerExtendRetractMotorConfig; 

    FollowerExtendRetractMotorConfig
    .SmartCurrentLimit(IntakeConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
    .Follow(m_ExtendRetractMotor, true);  // invert = true

    m_followerExtendRetractMotor.Configure(
      FollowerExtendRetractMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

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