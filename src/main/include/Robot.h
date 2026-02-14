// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "choreo/Choreo.h"



#include "choreo/Choreo.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand;

  RobotContainer m_container;

// Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive  
std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory = 
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("myTrajectory");
};
