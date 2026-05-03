#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <constants/HardwareConstants.h>
#include <constants/HoodConstants.h>

#include <rev/SparkMax.h>

#include <networktables/DoubleTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>


class HoodSubsystem : public frc2::SubsystemBase {
    public:
    HoodSubsystem();
    void GoToAngle(units::degree_t angle);
    frc2::CommandPtr GoToAngleCommand(units::degree_t angle); // fixme - might need t remove later
    frc2::CommandPtr GoToAngleCommand();
    void SetTargetAngle(units::turn_t angle);
    frc2::CommandPtr SetTargetAngleCommand(units::degree_t angle);

    frc2::CommandPtr RetractToLimitCommand();
    frc2::CommandPtr ExtendToLimitCommand();

    void Periodic() override;

    private:
    units::degrees_per_second_t GetAngleVelocity();
    units::degree_t m_targetAngle{HoodConstants::kMaxAngle};

    rev::spark::SparkMax m_angleMotor{HardwareConstants::kShooterHoodID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_angleController = m_angleMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_angleEncoder = m_angleMotor.GetEncoder();

    nt::DoublePublisher m_hoodAnglePub;
    nt::DoublePublisher m_angleMotorVoltagePub;
    nt::DoublePublisher m_angleMotorCurrentPub;
    nt::DoublePublisher m_hoodAngleVelocityPub;

    bool m_isCalibrated = false;
};