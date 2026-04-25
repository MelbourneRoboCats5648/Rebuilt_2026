#pragma once

#include <frc2/command/SubsystemBase.h>

#include <subsystems/FlyWheelSubsystem.h>
#include <subsystems/HoodSubsystem.h>
#include <subsystems/FeederSubsystem.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
    ShooterSubsystem(FlyWheelSubsystem& flyWheel, HoodSubsystem& hood, FeederSubsystem& feeder);

    private:
    FlyWheelSubsystem& m_flyWheel;
    HoodSubsystem& m_hood;
    FeederSubsystem& m_feeder;

};