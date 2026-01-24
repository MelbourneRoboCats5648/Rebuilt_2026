#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
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

public: 
    void StopMotors();
    void SetState(frc::SwerveModuleState state);
    units::meters_per_second_t GetSpeed();
    frc::Rotation2d GetAngle();
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

private:
    TalonFX m_speedMotor;
    TalonFX m_directionMotor;
    CANcoder m_directionEncoder;

    frc::ProfiledPIDController<radians> m_directionController{
        DirectionMotor::kP, DirectionMotor::kI, DirectionMotor::kD,
        { DirectionMotor::kMaxVel, DirectionMotor::kMaxAcc }
    };

    void SetSpeedMotorConfig();
    void SetDirectionMotorConfig();
    void SetDirectionEncoderConfig(turn_t magOffset);
    void ResetModulePosition();
    

};