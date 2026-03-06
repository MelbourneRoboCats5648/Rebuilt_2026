#pragma once

#include <rev/SparkMax.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>

#include <constants/IntakeConstants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    /* constructor */
    IntakeSubsystem();

private:
    turns_per_second_t CalculateIntakeSpeed(meters_per_second_t forwardRobotSpeed);

    rev::spark::SparkMax m_ExtendRetractMotor;
    rev::spark::SparkClosedLoopController m_extendRetractController = m_ExtendRetractMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_extendRetractEncoder = m_ExtendRetractMotor.GetEncoder();


    rev::spark::SparkMax m_followerExtendRetractMotor;

    rev::spark::SparkMax m_intakeMotor;
    rev::spark::SparkClosedLoopController m_intakeController = m_intakeMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

};
