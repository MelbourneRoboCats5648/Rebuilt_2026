#pragma once
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/PIDController.h>

class ChoreoController {
public:
ChoreoController();
void FollowTrajectory(const choreo::SwerveSample& sample, frc::Pose2d);



private:

    frc::PIDController xController{0.0, 0.0, 0.0};
    frc::PIDController yController{0.0, 0.0, 0.0};
    frc::PIDController headingController{0.0, 0.0, 0.0};

};