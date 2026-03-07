#include <drive/ChoreoController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>


ChoreoController::ChoreoController() {
    m_xController.SetTolerance(XYController::kTolerance.value());
    m_yController.SetTolerance(XYController::kTolerance.value());
    m_headingController.SetTolerance(ThetaController::kPositionTolerance.value(), ThetaController::kVelocityTolerance.value());

    m_headingController.EnableContinuousInput(-M_PI, M_PI);
};

frc::ChassisSpeeds ChoreoController::FollowTrajectory(const choreo::SwerveSample &sample, frc::Pose2d pose) {
    units::meters_per_second_t xFeedback{
        m_headingController.Calculate(pose.X().value(), sample.x.value())
    };
    units::meters_per_second_t yFeedback{
        m_yController.Calculate(pose.Y().value(), sample.y.value())
    };
    units::radians_per_second_t headingFeedback{    
        m_headingController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())
    };

    frc::ChassisSpeeds speeds{
        sample.vx + xFeedback,
        sample.vy + yFeedback,
        sample.omega + headingFeedback
    };

    return speeds;
}

void ChoreoController::Reset()
{
    m_xController.Reset();
    m_yController.Reset();
    m_headingController.Reset();
}

bool ChoreoController::AtSetpoint() {
    return m_xController.AtSetpoint() && m_yController.AtSetpoint() && m_headingController.AtSetpoint();
}

frc::PIDController& ChoreoController::getXController(){
    return m_xController;
}

frc::PIDController& ChoreoController::getYController(){
    return m_yController;
}

frc::PIDController& ChoreoController::getHeadingController(){
    return m_headingController;
}