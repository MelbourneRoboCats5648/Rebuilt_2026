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
#include <frc/controller/ProfiledPIDController.h>

using namespace units::velocity;
using namespace units::angle;
using namespace units::length;
using namespace units::angular_velocity;

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::hardware;

class ShooterSubsystem : public frc2::SubsystemBase {

    public:
        ShooterSubsystem();
        units::turns_per_second_t CalculateFlyWheelSpeed(meter_t distance, degree_t angle);

        frc2::CommandPtr SetTargetVelocityCommand(units::turns_per_second_t angularVelocity);

        void Shoot(units::volt_t volts);
        void GoToAngle(units::turn_t angle);
        void ShootAngularVelocity(units::turns_per_second_t angularVelocity);

        void SetTargetVelocity(units::turns_per_second_t velocity);
        units::turns_per_second_t GetTargetVelocity() const;

        units::meter_t DistanceToHub(frc::Pose2d robotPose);

        void Periodic() override;

        frc2::CommandPtr RetractToLimitCommand();

    private:
        turns_per_second_t GetAngleVelocity();
        frc2::CommandPtr DefaultShootCommand();

        meters_per_second_t CalculateBallSpeed(meter_t distance, degree_t angle);
        meters_per_second_t AdjustedBallSpeed(meters_per_second_t actualSpeed); // based on measurement of the 'theoretical ball speed' found in function above
        meter_t CalculateDistanceBetweenPoints(frc::Translation2d p1, frc::Translation2d p2);

        TalonFXConfiguration createMotorConfig();

        TalonFX m_motor;
        TalonFX m_follower;

        rev::spark::SparkMax m_angleMotor{HardwareConstants::kShooterHoodID, rev::spark::SparkMax::MotorType::kBrushless};
        rev::spark::SparkClosedLoopController m_angleController = m_angleMotor.GetClosedLoopController();
        rev::spark::SparkRelativeEncoder m_angleEncoder = m_angleMotor.GetEncoder();
        
        units::turns_per_second_t m_targetVelocity{0_tps};

        nt::DoublePublisher m_rotorVelPub;
        nt::DoublePublisher m_motorWheelVelPub;
        nt::DoublePublisher m_followerMotorWheelVelPub;
        nt::DoublePublisher m_motorCurrentPub;
        nt::DoublePublisher m_followerMotorCurrentPub;

        nt::DoublePublisher m_shooterAnglePub;
        nt::DoublePublisher m_angleMotorVoltagePub;
        nt::DoublePublisher m_angleMotorCurrentPub;

        // shooter speed debugging
        nt::DoublePublisher m_requiredSpedPub;
        nt::DoublePublisher m_adjustedSpeedPub;
        
        double m_requiredSpeed;
        double m_adjustedSpeed;
        
};