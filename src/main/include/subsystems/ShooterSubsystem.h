#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::configs;

class ShooterSubsystem : public frc2::SubsystemBase {

public:
    ShooterSubsystem(int motorID);


private:

    ctre::phoenix6::hardware::TalonFX m_motor;
    TalonFXConfiguration createMotorConfig();
};