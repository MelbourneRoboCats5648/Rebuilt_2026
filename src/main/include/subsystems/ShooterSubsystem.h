#pragma once

#include <frc2/command/SubsystemBase.h>

#include <subsystems/FlyWheelSubsystem.h>
#include <subsystems/HoodSubsystem.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/DriveSubsystem.h>

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
    ShooterSubsystem(DriveSubsystem& drive, FlyWheelSubsystem& flyWheel, HoodSubsystem& hood, FeederSubsystem& feeder, IntakeSubsystem& intake);
    void Periodic() override;
    frc2::CommandPtr ShootCommand();
    frc2::CommandPtr ShootCommandWithHood();
    frc2::CommandPtr ShootCommandWithFeeder(units::second_t feedTime);

    ShootSolution CompensateShootSolutionForRobotVelocity(ShootSolution ballSolution, meters_per_second_t robotRadialSpeed);
    ShootOnTheMoveSolution CompensateYawForTangentialSpeed(ShootSolution solution, units::meters_per_second_t robotTangentialSpeed);

    private:
    FlyWheelSubsystem& m_flyWheel;
    HoodSubsystem& m_hood;
    FeederSubsystem& m_feeder;
    IntakeSubsystem& m_intake;
    DriveSubsystem& m_drive;

};