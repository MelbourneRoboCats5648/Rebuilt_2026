#pragma once

#include <constants/ClimbConstants.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc2/command/CommandPtr.h>
#include <constants/HardwareConstants.h>

#include <frc/controller/ProfiledPIDController.h>

#include <networktables/DoubleTopic.h>

class ClimbSubsystem : public frc2::SubsystemBase{
    public:
    ClimbSubsystem();

    void Periodic() override;

    frc2::CommandPtr RetractCommand();
    frc2::CommandPtr ExtendCommand();

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkMax m_followerMotor;

    nt::DoublePublisher m_positionPub;

    //frc::ProfiledPIDController<units::meter> m_controller;

};
