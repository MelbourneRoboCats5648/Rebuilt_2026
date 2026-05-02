#pragma once

#include <frc2/command/SubsystemBase.h>

#include <subsystems/FlyWheelSubsystem.h>
#include <subsystems/HoodSubsystem.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/DriveSubsystem.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
    ShooterSubsystem(DriveSubsystem& drive, FlyWheelSubsystem& flyWheel, HoodSubsystem& hood, FeederSubsystem& feeder, IntakeSubsystem& intake);
    void Periodic() override;
    frc2::CommandPtr ShootCommand();
    frc2::CommandPtr ShootCommandWithHood();
    frc2::CommandPtr ShootCommandWithFeeder(units::second_t feedTime);

    private:
    FlyWheelSubsystem& m_flyWheel;
    HoodSubsystem& m_hood;
    FeederSubsystem& m_feeder;
    IntakeSubsystem& m_intake;
    DriveSubsystem& m_drive;

};