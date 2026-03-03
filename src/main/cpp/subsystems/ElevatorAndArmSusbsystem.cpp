#include "subsystems/ElevatorAndArmSubsystem.h"

using namespace ElevatorConstants;
using namespace ArmConstants;

ElevatorAndArmSubsystem::ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub, ArmSubsystem& armSub) 
  : m_elevatorSubsystem(elevatorSub),
    m_armSubsystem(armSub) {
}

void ElevatorAndArmSubsystem::MoveArm(double speed) {
  m_armSubsystem.MoveArm(speed);
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveUp() {
  return m_elevatorSubsystem.MoveUpCommand()
  .AndThen(m_armSubsystem.MoveUpCommand())
   .FinallyDo([this] {m_armSubsystem.StopMotor();});
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveDown() {
  return m_elevatorSubsystem.MoveDownCommand()
  .AndThen(m_armSubsystem.MoveDownCommand())
   .FinallyDo([this] {m_armSubsystem.StopMotor();});
}

frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveUp() {
  return m_elevatorSubsystem.MoveUpCommand();
 }

frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveDown() {
  return m_elevatorSubsystem.MoveDownCommand();
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveUp() {
  return m_armSubsystem.MoveUpCommand()
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveDown() {
  return m_armSubsystem.MoveDownCommand()
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveToAngle(units::turn_t armGoal) {
    return m_armSubsystem.MoveToAngleCommand(armGoal);
}

frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveToHeight(units::meter_t elevGoal) {
    return m_elevatorSubsystem.MoveToHeightCommand(elevGoal);           
}

units::meter_t ElevatorAndArmSubsystem::ElevatorGetHeight() {
    return m_elevatorSubsystem.GetElevatorHeight();
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveToLevel(Level level) { 
  ElevatorArmGoal goal;
  switch (level) {
    case DEFAULT: return DefaultPositionCommand();
    case L1: goal = ElevatorAndArmConstants::kLevel1Goal; break;
    case L2: goal = ElevatorAndArmConstants::kLevel2Goal; break;
    case L3: goal = ElevatorAndArmConstants::kLevel3Goal; break;
    default: return frc2::InstantCommand([]{}).ToPtr(); // do nothing - for correctness
  }

  return m_elevatorSubsystem.MoveToHeightCommand(goal.elevator)
            .AlongWith(m_armSubsystem.MoveToAngleCommand(goal.arm))
            .AndThen(frc2::InstantCommand([this, level] { m_level = level; }).ToPtr());
}

//For collecting coral, if elevator height is not high enough, 
//arm will not be able to rotate and then elevator move down to collect coral, 
//so in this case the arm must move upwards
frc2::CommandPtr ElevatorAndArmSubsystem::CollectCoral() {
  return
    DefaultPositionCommand() // ensure that we're in the default position before collecting
    .AndThen(
      m_elevatorSubsystem.m_secondStage.MoveToHeightCommand(ElevatorConstants::kInitSecondStageHeight)
    )
    .AndThen(
      DefaultPositionCommand()
    );
}

frc2::CommandPtr ElevatorAndArmSubsystem::PlaceCoral() {
  return m_elevatorSubsystem.m_secondStage.MoveUpBy(kElevatorPlaceCoral)
          .AlongWith(m_armSubsystem.RotateBy(kArmPlaceCoral));
}

frc2::CommandPtr ElevatorAndArmSubsystem::DefaultPositionCommand() {
  return 
    m_elevatorSubsystem.MoveToHeightCommand(ElevatorAndArmConstants::kDefaultGoal.elevator)
    .AlongWith(
      frc2::cmd::WaitUntil([this] {
        if (m_elevatorSubsystem.m_secondStage.GetHeight() > kElevatorClearanceThreshold)
        {
          return true;
        }
        else
        {
          return false;
        }
      }).AndThen(m_armSubsystem.MoveToAngleCommand(ElevatorAndArmConstants::kDefaultGoal.arm))
    ).AndThen(frc2::InstantCommand([this] { m_level = DEFAULT; }).ToPtr());
}

void ElevatorAndArmSubsystem::LogEncoderOutputs()
{
  frc::SmartDashboard::PutNumber("ElevatorAndArmSubsystem/ArmPosition", m_armSubsystem.GetArmAngle().value());
}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  LogEncoderOutputs();
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
