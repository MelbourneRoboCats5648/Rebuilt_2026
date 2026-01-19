#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>
#include <frc/controller/ProfiledPIDController.h>

#include <constants/DriveConstants.h>

using namespace units::angle;
using namespace ctre::phoenix6::hardware;
using namespace DriveModuleConstants;

class DriveModule {
public:
    DriveModule(
        int speedMotorID, int directionMotorID, int directionEncoderID,
        turn_t magOffset
    );

private:
    TalonFX m_speedMotor;
    TalonFX m_directionMotor;
    CANcoder m_directionEncoder;

    frc::ProfiledPIDController<radians> m_directionController{
        DirectionController::kP, DirectionController::kI, DirectionController::kD,
        { DirectionController::kMaxVel, DirectionController::kMaxAcc }
    };

    void SetSpeedMotorConfig();
    void SetDirectionMotorConfig();
    void SetDirectionEncoderConfig(turn_t magOffset);

};