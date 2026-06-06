#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include <frc/Timer.h>
#include <rev/SparkMax.h>

class FeederSubsystem : public frc2::SubsystemBase {
public:
    FeederSubsystem();

    void Feed();
    void Stop();
    bool IsStalling();

    frc2::CommandPtr FeedCommand(); // will run indefinitely until cancelled
    // frc2::CommandPtr IsStalling();

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMax m_leaderSideMotor;
    rev::spark::SparkMax m_followerSideMotor;

    // frc::Timer timer; 
};
