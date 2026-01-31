#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"


ShooterSubsystem::ShooterSubsystem(int motorID)
: m_motor(motorID, "rio")
{
    TalonFXConfiguration motorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(motorConfig);
}

TalonFXConfiguration ShooterSubsystem::createMotorConfig(){

    TalonFXConfiguration motorConfig;
    motorConfig.Slot0.kP = ShooterConstants::motor::kP;
    motorConfig.Slot0.kI = ShooterConstants::motor::kI;
    motorConfig.Slot0.kD = ShooterConstants::motor::kD;

    return motorConfig;
    
}