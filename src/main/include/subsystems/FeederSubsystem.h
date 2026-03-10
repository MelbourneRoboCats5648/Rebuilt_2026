#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class FeederSubsystem : public frc2::SubsystemBase {
public:
    FeederSubsystem();

    void Feed();
    void Stop();

    frc2::CommandPtr FeedCommand(); // will run indefinitely until cancelled

private:
    rev::spark::SparkMax m_motor;
};
