#pragma once

#include <constants/ClimbConstants.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc2/command/CommandPtr.h>
#include <constants/HardwareConstants.h>


#include <frc/controller/ProfiledPIDController.h>

class ClimbSubsystem : public frc2::SubsystemBase{
    public:
    ClimbSubsystem();

    frc2::CommandPtr ClimbUpCommand();
    frc2::CommandPtr ClimbDownCommand();

    void ResetEncoder();
    void ResetMotor();
    void LimitSwitchActivation();

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMax m_followerMotor;

    frc::ProfiledPIDController<units::meter> m_controller;

};
