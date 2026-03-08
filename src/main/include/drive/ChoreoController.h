#pragma once
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/PIDController.h>

#include <constants/DriveConstants.h>

using namespace DrivetrainConstants::Autonomous;

class ChoreoController {
public:
    ChoreoController();
    frc::ChassisSpeeds FollowTrajectory(const choreo::SwerveSample& sample, frc::Pose2d pose);

    void Reset();

    bool AtSetpoint();

private:
    frc::PIDController m_xController{
        XYController::kP, XYController::kI, XYController::kD
    }; // metre
    frc::PIDController m_yController{
        XYController::kP, XYController::kI, XYController::kD
    }; // metre
    frc::PIDController m_headingController{
        ThetaController::kP, ThetaController::kI, ThetaController::kD
    }; // radian
};