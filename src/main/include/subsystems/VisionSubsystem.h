#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <networktables/StructTopic.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <constants/VisionConstants.h>

class VisionSubsystem : public frc2::SubsystemBase {
public:
    VisionSubsystem(frc::SwerveDrivePoseEstimator<4>& estimator);

    void Periodic() override;

private:
    frc::SwerveDrivePoseEstimator<4>& m_estimator;

    photon::PhotonCamera m_camera{VisionConstants::kCameraName};
    photon::PhotonPoseEstimator m_photonEstimator{
        VisionConstants::kTagLayout,
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants::kCameraLocation
    };
    // photon::PhotonPipelineResult m_latestResult;

    Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d pose);

    nt::StructPublisher<frc::Pose2d> m_posePublisher;
};