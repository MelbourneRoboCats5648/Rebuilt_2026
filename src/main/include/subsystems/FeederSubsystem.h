#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <constants/FeederConstants.h>

#include <rev/SparkMax.h>

class FeederSubsystem : public frc2::SubsystemBase {
public:
    FeederSubsystem();

    void Feed();
    void ReverseFeed();
    void Stop();
    bool IsStalling();

    frc2::CommandPtr FeedCommand(); // will run indefinitely until cancelled
    frc2::CommandPtr IncreaseFeederVoltageDifference();
    frc2::CommandPtr DecreaseFeederVoltageDifference();
    frc2::CommandPtr ReverseFeedCommand();

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMax m_leftSideMotor;
    rev::spark::SparkMax m_rightSideMotor;
};
