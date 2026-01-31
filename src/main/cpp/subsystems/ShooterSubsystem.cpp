#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"

using namespace ctre::phoenix6::controls;

ShooterSubsystem::ShooterSubsystem(int motorID, int followerID)
: m_motor(motorID, "rio"),
  m_follower(followerID, "rio")
{
    TalonFXConfiguration motorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(motorConfig);
    m_follower.GetConfigurator().Apply(motorConfig);

    m_follower.SetControl(Follower{m_motor.GetDeviceID(), false});
}

TalonFXConfiguration ShooterSubsystem::createMotorConfig(){

    TalonFXConfiguration motorConfig;
    motorConfig.Slot0.kP = ShooterConstants::motor::kP;
    motorConfig.Slot0.kI = ShooterConstants::motor::kI;
    motorConfig.Slot0.kD = ShooterConstants::motor::kD;

    return motorConfig;
    
}
