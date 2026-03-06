#pragma once
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/PIDController.h>

class ChoreoController {
public:
    ChoreoController();
    frc::ChassisSpeeds FollowTrajectory(const choreo::SwerveSample& sample, frc::Pose2d pose);

    void Reset();

    frc::PIDController& getXController();
    frc::PIDController& getYController();
    frc::PIDController& getHeadingController();

private:
    // fixme - need to update these PID values
    frc::PIDController m_xController{0.0, 0.0, 0.0};
    frc::PIDController m_yController{0.0, 0.0, 0.0};
    frc::PIDController m_headingController{0.0, 0.0, 0.0};
};