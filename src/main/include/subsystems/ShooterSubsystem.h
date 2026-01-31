#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::hardware;

class ShooterSubsystem : public frc2::SubsystemBase {

public:
    ShooterSubsystem(int motorID, int followerID);


private:

    TalonFX m_motor;
    TalonFX m_follower;
    TalonFXConfiguration createMotorConfig();
};