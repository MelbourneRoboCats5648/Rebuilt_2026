#pragma once

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>


class ElevatorAndArmSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub,
    ArmSubsystem& armSub);

  void MoveArm(double speed);
  void MoveSecondStage(double speed);

  frc2::CommandPtr MoveUp();
  frc2::CommandPtr MoveDown();
  frc2::CommandPtr ElevatorMoveUp();
  frc2::CommandPtr ElevatorMoveDown();
  frc2::CommandPtr ArmMoveUp();
  frc2::CommandPtr ArmMoveDown();
  frc2::CommandPtr ArmMoveToAngle(units::turn_t armGoal);
  frc2::CommandPtr ElevatorMoveToHeight(units::meter_t elevGoal);
  units::meter_t ElevatorGetHeight();
  frc2::CommandPtr MoveToLevel(Level level);
  frc2::CommandPtr CollectCoral();
  frc2::CommandPtr PlaceCoral();

  frc2::CommandPtr DefaultPositionCommand();

  void LogEncoderOutputs();
  
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ElevatorSubsystem& m_elevatorSubsystem;
  ArmSubsystem& m_armSubsystem;

  Level m_level = COLLECT;
};

