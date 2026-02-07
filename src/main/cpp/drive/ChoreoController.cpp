#include <drive/ChoreoController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>


ChoreoController::ChoreoController() {
    headingController.EnableContinuousInput(-M_PI, M_PI);
};

void ChoreoController::FollowTrajectory(const choreo::SwerveSample &sample, frc::Pose2d pose) {
    units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
    units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};
    units::radians_per_second_t headingFeedback{    
        headingController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())
        };

frc::ChassisSpeeds speeds{
    sample.vx + xFeedback,
    sample.vy + yFeedback,
    sample.omega + headingFeedback
};


};
