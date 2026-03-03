#include "subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() {

}

units::meter_t ElevatorSubsystem::GetElevatorHeight() {
  return m_firstStage.GetHeight() + m_secondStage.GetHeight();
}

frc2::CommandPtr ElevatorSubsystem::MoveUpCommand() {
  return
    m_firstStage.MoveUpCommand()
    .AlongWith(m_secondStage.MoveUpCommand());
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  return
    m_firstStage.MoveDownCommand()
    .AlongWith(m_secondStage.MoveDownCommand());
}

frc2::CommandPtr ElevatorSubsystem::MoveToHeightCommand(units::meter_t heightGoal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  if(heightGoal <= ElevatorConstants::kMaxSecondStageHeight) {
    return
      m_secondStage.MoveToHeightCommand(heightGoal)
      .AlongWith(m_firstStage.MoveToHeightCommand(0_m));
  }
  else {
    return
      m_secondStage.MoveToHeightCommand(ElevatorConstants::kMaxSecondStageHeight)
      .AlongWith(m_firstStage.MoveToHeightCommand(heightGoal - ElevatorConstants::kMaxSecondStageHeight));
  }
}

//To move down supply a negative
// frc2::CommandPtr ElevatorSubsystem::MoveUpBy(units::meter_t height) {

//       units::meter_t moveGoal = (GetElevatorHeight() + height);
//       return MoveToHeightCommand(moveGoal);
// }

void ElevatorSubsystem::Periodic() {
  
}

void ElevatorSubsystem::SimulationPeriodic() {
  
}
