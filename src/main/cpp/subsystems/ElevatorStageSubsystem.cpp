#include <subsystems/ElevatorStageSubsystem.h>
#include <algorithm>

ElevatorStageSubsystem::ElevatorStageSubsystem(
  std::string name,
  units::meter_t minLimit, units::meter_t maxLimit,
  units::meter_t initHeight, units::meter_t resetHeight,
  units::meter_t distancePerTurn,
  PIDConstants pidConst, ElevatorFeedforwardConstants ffConst,
  frc::TrapezoidProfile<units::meter>::Constraints pidProfile,
  int canID, int limitSwitchPin, int followerID
) : m_name(name),
    m_limitSwitch(limitSwitchPin),
    m_motor(canID, rev::spark::SparkMax::MotorType::kBrushless),
    m_encoder(m_motor.GetEncoder()),
    m_controller(pidConst.kP, pidConst.kI, pidConst.kD, pidProfile, ElevatorConstants::kDt),
    m_feedforward(ffConst.kS, ffConst.kG, ffConst.kV, ffConst.kA),
    m_minLimit(minLimit), m_maxLimit(maxLimit), m_resetHeight(resetHeight),
    m_gearRatio(ElevatorConstants::gearBoxGearRatio * distancePerTurn.value())
 {
    rev::spark::SparkMaxConfig motorConfig;
    motorConfig
      .SmartCurrentLimit(ElevatorConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
    motorConfig.softLimit
      .ForwardSoftLimit(maxLimit.value()).ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(minLimit.value()).ReverseSoftLimitEnabled(true);
    motorConfig.encoder
      .PositionConversionFactor(m_gearRatio)
      .VelocityConversionFactor(m_gearRatio);
    m_motor.Configure(
      motorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );
    m_encoder.SetPosition(initHeight.value());

    if (followerID >= 0) {
      /* we have a follower motor controller, let's set it up */
      rev::spark::SparkMax followerMotor(followerID, rev::spark::SparkMax::MotorType::kBrushless);
      rev::spark::SparkMaxConfig followerConfig;
      followerConfig
        .SmartCurrentLimit(ElevatorConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
        .Follow(m_motor, true); // invert = true
      followerMotor.Configure(
        followerConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kPersistParameters
      );
    }
    
    /* set up PID controller */
    m_controller.SetTolerance(ElevatorConstants::kElevatorPositionTolerance, ElevatorConstants::kElevatorVelocityTolerance);

    SetDefaultCommand(HoldPositionCommand());
}

units::meter_t ElevatorStageSubsystem::GetHeight() {
  return units::meter_t(m_encoder.GetPosition());
}

units::meters_per_second_t ElevatorStageSubsystem::GetVelocity() {
  return units::meters_per_second_t(m_encoder.GetVelocity() / 60.0); // GetVelocity() returns metres per minute
}

bool ElevatorStageSubsystem::IsGoalReached() {
  return m_controller.AtGoal();
}

void ElevatorStageSubsystem::SetpointControl() {
  double output = 
    m_controller.Calculate(GetHeight()) // profiled PID
    + (m_feedforward.Calculate(m_controller.GetSetpoint().velocity) / frc::RobotController::GetBatteryVoltage()); // feedforward, normalised
  output = std::clamp(output, -ElevatorConstants::maxOutput, ElevatorConstants::maxOutput); // clamp to [-maxOutput, maxOutput]

  m_motor.Set(output);
}

void ElevatorStageSubsystem::VelocityControl(units::meters_per_second_t velocity) {
  m_motor.SetVoltage(m_feedforward.Calculate(velocity));
}

frc2::CommandPtr ElevatorStageSubsystem::SetpointControlCommand() {
  return Run([this] { SetpointControl(); });
}

void ElevatorStageSubsystem::ResetController() {
  m_controller.Reset(GetHeight(), GetVelocity());
}

frc2::CommandPtr ElevatorStageSubsystem::HoldPositionCommand() {
  return StartRun(
    [this] { ResetController(); m_controller.SetGoal(GetHeight()); },
    [this] { SetpointControl(); }
  );
}

void ElevatorStageSubsystem::ResetMotor() {
  m_motor.Set(0);
}

void ElevatorStageSubsystem::ResetEncoder() {
  m_encoder.SetPosition(m_resetHeight.value());
}

void ElevatorStageSubsystem::OnLimitSwitchActivation() {
  ResetMotor();
  ResetEncoder();
}

frc2::CommandPtr ElevatorStageSubsystem::MoveUpCommand() {
  return Run([this] { m_motor.Set(0.1); }); // default command will take over upon cancellation
}

frc2::CommandPtr ElevatorStageSubsystem::MoveDownCommand() {
  return Run([this] { m_motor.Set(-0.1); });
}

frc2::CommandPtr ElevatorStageSubsystem::MoveToHeightCommand(units::meter_t heightGoal) {
  return
    StartRun(
      [this, heightGoal] { ResetController(); m_controller.SetGoal(heightGoal); },
      [this] { SetpointControl(); }
    )
    .Until([this] { return IsGoalReached(); });
}

frc2::CommandPtr ElevatorStageSubsystem::MoveUpBy(units::meter_t height) {
  return
    StartRun(
      [this, height] { ResetController(); m_controller.SetGoal(GetHeight() + height); },
      [this] { SetpointControl(); }
    )
    .Until([this] { return IsGoalReached(); });
}

void ElevatorStageSubsystem::SetPower(double power) {
  m_motor.Set(power);
}

void ElevatorStageSubsystem::Periodic() {
  auto setpoint = m_controller.GetSetpoint();
  frc::SmartDashboard::PutNumber(m_name + "/setpoint/position", setpoint.position.value());
  frc::SmartDashboard::PutNumber(m_name + "/setpoint/velocity", setpoint.velocity.value());

  auto goal = m_controller.GetGoal();
  frc::SmartDashboard::PutNumber(m_name + "/goal/position", goal.position.value());
  frc::SmartDashboard::PutNumber(m_name + "/goal/velocity", goal.velocity.value());

  frc::SmartDashboard::PutNumber(m_name + "/encoder/position", GetHeight().value());
  frc::SmartDashboard::PutNumber(m_name + "/encoder/velocity", GetVelocity().value());
}

void ElevatorStageSubsystem::SimulationPeriodic() {
  
}
