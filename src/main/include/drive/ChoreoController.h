#pragma once
#include <frc2/command/SubsystemBase.h>
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/HolonomicDriveController.h>

class ChoreoController : public frc2::SubsystemBase {
public:
ChoreoController();
void FollowTrajectory(const choreo::SwerveSample& sample);

private:

    frc::PIDController xController{0.0, 0.0, 0.0};
    frc::PIDController yController{0.0, 0.0, 0.0};
    frc::PIDController headingController{0.0, 0.0, 0.0};

};