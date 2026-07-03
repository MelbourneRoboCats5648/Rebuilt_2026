#pragma once

#include <frc2/command/SubsystemBase.h>

#include <subsystems/FlyWheelSubsystem.h>
#include <subsystems/HoodSubsystem.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/DriveSubsystem.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>

struct ShootSolution{
    degree_t angle;
    meters_per_second_t speed;
};

struct ShootOnTheMoveSolution{
    ShootSolution shootSolution;
    degree_t yawAngle;
};

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
    ShooterSubsystem(DriveSubsystem& drive, IntakeSubsystem& intake);
    void Periodic() override;
    frc2::CommandPtr ShootCommand();
    frc2::CommandPtr ShootCommandWithHood();
    frc2::CommandPtr ShootCommandWithFeeder(units::second_t feedTime);

    frc2::CommandPtr RetractHoodToLimitCommand();
    frc2::CommandPtr SetHoodTargetAngleCommand(units::degree_t angle);

    frc2::CommandPtr SetFlywheelVelocityCommand(units::turns_per_second_t angularVelocity);

    frc2::CommandPtr IncreaseFeederVoltageDifference();
    frc2::CommandPtr DecreaseFeederVoltageDifference();

    ShootSolution CompensateForRadialSpeed(ShootSolution ballSolution, meters_per_second_t robotRadialSpeed);
    ShootOnTheMoveSolution CompensateYawForTangentialSpeed(ShootSolution solution, units::meters_per_second_t robotTangentialSpeed);

    private:
    FlyWheelSubsystem m_flyWheel;
    HoodSubsystem m_hood;
    FeederSubsystem m_feeder;

    IntakeSubsystem& m_intake;
    DriveSubsystem& m_drive;

    meters_per_second_t CalculateRequiredBallSpeed(meter_t distance, degree_t angle);

    nt::DoublePublisher m_staticAnglePub;
    nt::DoublePublisher m_staticVelocityPub;

    nt::DoublePublisher m_radialCompensatedAnglePub;
    nt::DoublePublisher m_radialCompensatedVelocityPub;
    
    nt::DoublePublisher m_radialCompensatedAngleDeltaPub;
    nt::DoublePublisher m_radialCompensatedVelocityDeltaPub;

    nt::DoublePublisher m_moveAnglePub;
    nt::DoublePublisher m_moveVelocityPub;
    nt::DoublePublisher m_moveYawPub;

    nt::DoublePublisher m_moveCompensatedAnglePub;
    nt::DoublePublisher m_moveFlywheelVelocityPub;
    nt::DoublePublisher m_moveCompensatedYawPub;
};