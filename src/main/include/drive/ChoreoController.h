#pragma once
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/PIDController.h>

class ChoreoController {
public:
    ChoreoController();
    void FollowTrajectory(const choreo::SwerveSample& sample, frc::Pose2d);

    frc::PIDController& getHeadingController();

private:
    frc::PIDController m_xController{0.0, 0.0, 0.0};
    frc::PIDController m_yController{0.0, 0.0, 0.0};
    frc::PIDController m_headingController{0.0, 0.0, 0.0};
};