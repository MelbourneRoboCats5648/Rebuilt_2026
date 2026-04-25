#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "units/velocity.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/angular_velocity.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

#include <constants/HardwareConstants.h>

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <subsystems/DriveSubsystem.h>
#include <constants/FlyWheelConstants.h>

using namespace units::velocity;
using namespace units::angle;
using namespace units::length;
using namespace units::angular_velocity;

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::hardware;

class FlyWheelSubsystem : public frc2::SubsystemBase {

    public:
        FlyWheelSubsystem(DriveSubsystem& drive);
        units::turns_per_second_t CalculateFlyWheelSpeed(meter_t distance, degree_t angle);

        frc2::CommandPtr SetTargetVelocityCommand(units::turns_per_second_t angularVelocity);

        void Shoot(units::volt_t volts);
        void ShootAngularVelocity(units::turns_per_second_t angularVelocity);

        void SetTargetVelocity(units::turns_per_second_t velocity);
        units::turns_per_second_t GetTargetVelocity() const;
        //void SetFlywheelVelocityAndAngle(meter_t distanceToTarget);

        void Periodic() override;

        frc2::CommandPtr IncreaseFlywheelVelocity();
        frc2::CommandPtr DecreaseFlywheelVelocity();
        frc2::CommandPtr ResetFlywheelVelocity();
        
        frc2::CommandPtr ShootCommand();

    private:
        meters_per_second_t CalculateBallSpeed(meter_t distance, degree_t angle);
        meters_per_second_t AdjustedBallSpeed(meters_per_second_t actualSpeed); // based on measurement of the 'theoretical ball speed' found in function above

        double m_scaleFlywheelVelocity = FlyWheelConstants::kDefaultFlywheelVelocityScaling; // default is scaling by unity (no change in speed)

        TalonFXConfiguration createMotorConfig();

        TalonFX m_motor;
        TalonFX m_follower;

        DriveSubsystem& m_drive; // for retrieving pose only; not required in commands

        units::turns_per_second_t m_targetVelocity{0_tps}; // flywheel velocity

        nt::DoublePublisher m_rotorVelPub;
        nt::DoublePublisher m_motorWheelVelPub;
        nt::DoublePublisher m_followerMotorWheelVelPub;
        nt::DoublePublisher m_motorCurrentPub;
        nt::DoublePublisher m_followerMotorCurrentPub;

        nt::DoublePublisher m_m_flyWheelVoltagePub;
        nt::DoublePublisher m_m_flyWheelTargetVelPub;

        // shooter speed debugging
        nt::DoublePublisher m_requiredSpedPub;
        nt::DoublePublisher m_adjustedSpeedPub;
        
        double m_requiredSpeed;
        double m_adjustedSpeed;        
};
