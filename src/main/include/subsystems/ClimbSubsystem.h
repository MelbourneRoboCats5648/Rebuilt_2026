#pragma once

#include <constants/ClimbConstants.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class ClimbSubsystem : public frc2::SubsystemBase{
    public:
    ClimbSubsystem(int motorCanID, int followerMotorCanID);

    frc2::CommandPtr ClimbUpCommand();
    frc2::CommandPtr MoveDownCommand();
    frc2::CommandPtr DefaultPosition();

    void ResetEncoder();
    void ResetMotor();
    void LimitSwitchActivation();

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMax m_followerMotor;

};
