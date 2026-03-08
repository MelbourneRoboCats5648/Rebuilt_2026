#include <subsystems/IntakeSubsystem.h>

#include <constants/HardwareConstants.h>

#include <rev/config/SparkMaxConfig.h>
#include <frc/RobotController.h>

#include <algorithm>

IntakeSubsystem::IntakeSubsystem() :
m_extendRetractMotor(HardwareConstants::kExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_followerExtendRetractMotor(HardwareConstants::kFollowerExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_intakeMotor(HardwareConstants::kIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_extendRetractFeedforward(IntakeConstants::extendRetract::kS, IntakeConstants::extendRetract::kG, IntakeConstants::extendRetract::kV, IntakeConstants::extendRetract::kA),
m_extendRetractPID(
    IntakeConstants::extendRetract::kP, IntakeConstants::extendRetract::kI, IntakeConstants::extendRetract::kD,
    {IntakeConstants::extendRetract::kMaxVelocity, IntakeConstants::extendRetract::kMaxAcceleration}
),
m_intakeFeedforward(IntakeConstants::intake::kS, IntakeConstants::intake::kV, IntakeConstants::intake::kA),
m_intakePID(IntakeConstants::intake::kP, IntakeConstants::intake::kI, IntakeConstants::intake::kD)
{
    //intake motor config
    rev::spark::SparkMaxConfig intakeMotorConfig;

    intakeMotorConfig
      .SmartCurrentLimit(IntakeConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);

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

    ExtendRetractMotorConfig.softLimit
      .ForwardSoftLimit(IntakeConstants::kExtendSoftLimit.value()).ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(IntakeConstants::kRetractSoftLimit.value()).ReverseSoftLimitEnabled(true);

    const double metresPerTurn = IntakeConstants::kExtendRetractSprocketDia * std::numbers::pi * IntakeConstants::kExtendRetractGearRatio;

    ExtendRetractMotorConfig.encoder
      .PositionConversionFactor(metresPerTurn)
      .VelocityConversionFactor(metresPerTurn / 60); // by default Spark Max returns RPM; we want to convert to m/s here (hence we divide by 60 too)
    
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

    m_extendRetractPID.SetTolerance(IntakeConstants::extendRetract::kPositionTolerance, IntakeConstants::extendRetract::kVelocityTolerance);
    m_intakePID.SetTolerance(IntakeConstants::intake::kTolerance.value());
}

void IntakeSubsystem::ConfigurePublishers()
{
    m_extendRetractPositionPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetract/Position").Publish();
    m_extendRetractVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetract/Velocity").Publish();
    m_extendRetractMotorCurrentPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetract/Current").Publish();
    m_followerExtendRetractMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetract/FollowerCurrent").Publish();
    m_extendRetractVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/ExtendRetract/Voltage").Publish();
    m_intakeVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/Motor/Velocity").Publish();
    m_intakeVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Intake/Motor/Voltage").Publish();
}

void IntakeSubsystem::Periodic()
{
    /* publish current state */
    m_extendRetractPositionPub.Set(GetPosition().value());
    m_extendRetractVelocityPub.Set(GetExtendRetractVelocity().value());
    m_extendRetractMotorCurrentPub.Set(m_extendRetractMotor.GetOutputCurrent());
    m_followerExtendRetractMotorCurrentPub.Set(m_followerExtendRetractMotor.GetOutputCurrent());
    m_intakeVelocityPub.Set(GetIntakeVelocity().value());
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

units::meter_t IntakeSubsystem::GetPosition() {
    return units::meter_t{m_extendRetractEncoder.GetPosition()};
}

units::meters_per_second_t IntakeSubsystem::GetExtendRetractVelocity() {
    return units::meters_per_second_t{m_extendRetractEncoder.GetVelocity()};
}

void IntakeSubsystem::SetPosition(units::meter_t position) {
    m_extendRetractPID.Reset(GetPosition(), GetExtendRetractVelocity());
    m_extendRetractPID.SetGoal(position);
}

void IntakeSubsystem::ExtendRetractControl() {
    units::volt_t output =
        units::volt_t{m_extendRetractPID.Calculate(GetPosition())}
        + m_extendRetractFeedforward.Calculate(m_extendRetractPID.GetSetpoint().velocity);
    m_extendRetractMotor.SetVoltage(output);
    m_extendRetractVoltagePub.Set(output.value());
}

bool IntakeSubsystem::IsAtPosition() {
    return m_extendRetractPID.AtGoal();
}

void IntakeSubsystem::StopExtendRetract() {
    m_extendRetractMotor.SetVoltage(0.0_V); // should set the motor to coast
    m_extendRetractVoltagePub.Set(0.0);
}

frc2::CommandPtr IntakeSubsystem::ExtendRetractCommand(units::meter_t position) {
    return StartRun(
        [this, position] { SetPosition(position); },
        [this] { ExtendRetractControl(); }
    ).Until([this] { return IsAtPosition(); })
    .FinallyDo([this] { StopExtendRetract(); });
}

void IntakeSubsystem::SetIntakeVelocity(units::turns_per_second_t velocity) {
    m_intakePID.Reset();
    m_intakePID.SetSetpoint(velocity.value());
}

units::turns_per_second_t IntakeSubsystem::GetIntakeVelocity() {
    return units::turns_per_second_t{m_intakeEncoder.GetVelocity() / 60.0}; // convert from RPM to tps
}

void IntakeSubsystem::IntakeControl() {
    units::volt_t output =
        units::volt_t{m_intakePID.Calculate(GetIntakeVelocity().value())}
        + m_intakeFeedforward.Calculate(units::turns_per_second_t{m_intakePID.GetSetpoint()});
    SetIntakeVoltage(output);
}

bool IntakeSubsystem::IsAtVelocity() {
    return m_intakePID.AtSetpoint();
}

void IntakeSubsystem::StopIntake() {
    m_intakeMotor.SetVoltage(0.0_V); // should set the motor to coast
}

frc2::CommandPtr IntakeSubsystem::IntakeCommand(units::turns_per_second_t velocity) {
    return StartRun(
        [this, velocity] { SetIntakeVelocity(velocity); },
        [this] { IntakeControl(); }
    ).FinallyDo([this] { StopIntake(); });
}

void IntakeSubsystem::SetIntakeVoltage(units::volt_t voltage) {
    m_intakeMotor.SetVoltage(voltage);
    m_intakeVoltagePub.Set(voltage.value());
}
