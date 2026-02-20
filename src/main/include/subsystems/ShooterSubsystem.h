#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "units/velocity.h"
#include "units/angle.h"
#include "units/length.h"

#include <ctre/phoenix6/TalonFX.hpp>


using namespace units::velocity;
using namespace units::angle;
using namespace units::length;

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::hardware;

class ShooterSubsystem : public frc2::SubsystemBase {

    public:
        ShooterSubsystem(int motorID, int followerID);
        meters_per_second_t CalculateShooterSpeed(meter_t distance, degree_t angle);

        frc2::CommandPtr ShootCommand(units::volt_t volts);

    private:


    TalonFX m_motor;
    TalonFX m_follower;
    TalonFXConfiguration createMotorConfig();
};