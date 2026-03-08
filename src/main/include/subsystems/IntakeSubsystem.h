#pragma once

#include <rev/SparkMax.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <constants/IntakeConstants.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    /* constructor */
    IntakeSubsystem();

    void SetPosition(units::meter_t position);
    units::meter_t GetPosition();
    units::meters_per_second_t GetExtendRetractVelocity();
    void ExtendRetractControl(); // run extend/retract motor
    bool IsAtPosition(); // check if extend/retract motor is at goal position
    void StopExtendRetract();

    frc2::CommandPtr ExtendRetractCommand(units::meter_t position);
    
    void SetIntakeVelocity(units::turns_per_second_t velocity);
    units::turns_per_second_t GetIntakeVelocity();
    void IntakeControl(); // run intake motor
    bool IsAtVelocity(); // check if intake motor is at goal velocity
    void StopIntake();

    void SetIntakeVoltage(units::volt_t voltage);
    void SetExtendRetractVoltage(units::volt_t voltage);

    frc2::CommandPtr IntakeCommand(units::turns_per_second_t velocity);

    void Periodic() override;

private:
    turns_per_second_t CalculateIntakeSpeed(meters_per_second_t forwardRobotSpeed);

    void ConfigurePublishers();

    rev::spark::SparkMax m_extendRetractMotor;
    rev::spark::SparkRelativeEncoder m_extendRetractEncoder = m_extendRetractMotor.GetEncoder();

    rev::spark::SparkMax m_followerExtendRetractMotor;

    rev::spark::SparkMax m_intakeMotor;
    rev::spark::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

    frc::ElevatorFeedforward m_extendRetractFeedforward;
    frc::ProfiledPIDController<units::meter> m_extendRetractPID; // with velocity and acceleration constraints; probably needed

    frc::SimpleMotorFeedforward<units::turns> m_intakeFeedforward;
    frc::PIDController m_intakePID; // NOTE: do we need profiling (i.e. acceleration constraints) for intake?

    // publishers
    nt::DoublePublisher m_extendRetractPositionPub;
    nt::DoublePublisher m_extendRetractVelocityPub;
    nt::DoublePublisher m_extendRetractMotorCurrentPub;
    nt::DoublePublisher m_followerExtendRetractMotorCurrentPub;
    nt::DoublePublisher m_intakeVelocityPub;
    nt::DoublePublisher m_intakeVoltagePub;
    nt::DoublePublisher m_extendRetractVoltagePub;
};
