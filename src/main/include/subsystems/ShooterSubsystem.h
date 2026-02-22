#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "units/velocity.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/angular_velocity.h"

#include <ctre/phoenix6/TalonFX.hpp>

#include <constants/HardwareConstants.h>

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>

using namespace units::velocity;
using namespace units::angle;
using namespace units::length;

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::hardware;

class ShooterSubsystem : public frc2::SubsystemBase {

    public:
        ShooterSubsystem();
        meters_per_second_t CalculateShooterSpeed(meter_t distance, degree_t angle);

        frc2::CommandPtr ShootCommand(units::volt_t volts);
        void Shoot(units::volt_t volts);
        void ShootAngularVelocity(units::turns_per_second_t angularVelocity);

        void Periodic() override;


    private:


    TalonFX m_motor;
    TalonFX m_follower;
    TalonFXConfiguration createMotorConfig();

    nt::DoublePublisher m_rotorVelPub;
    nt::DoublePublisher m_wheelVelPub;  
};