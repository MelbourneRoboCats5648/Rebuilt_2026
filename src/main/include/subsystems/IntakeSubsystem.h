#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <constants/IntakeConstants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    /* constructor */
    IntakeSubsystem();

private:
    turns_per_second_t CalculateIntakeSpeed(meters_per_second_t forwardRobotSpeed);
};
