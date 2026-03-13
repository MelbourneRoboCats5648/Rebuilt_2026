#include <subsystems/VisionSubsystem.h>

VisionSubsystem::VisionSubsystem(frc::SwerveDrivePoseEstimator<4>& estimator) : m_estimator(estimator) {
    m_photonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

    m_posePublisher = nt::NetworkTableInstance::GetDefault()
        .GetStructTopic<frc::Pose2d>("Vision/Pose").Publish();
}

void VisionSubsystem::Periodic() {
    // run each new pipeline result through our pose estimator
    for (const auto& result : m_camera.GetAllUnreadResults()) {
        auto estimatedPose = m_photonEstimator.Update(result); // estimated pose from this result

        if (estimatedPose) { // pose can be resolved
            frc::Pose2d pose = estimatedPose->estimatedPose.ToPose2d();
            m_posePublisher.Set(pose);
            auto stddevs = GetEstimationStdDevs(pose);
            m_estimator.AddVisionMeasurement(
                pose, estimatedPose->timestamp,
                {stddevs(0), stddevs(1), stddevs(2)}
            ); // update drivetrain pose estimator
        }
    }
}

/* from PhotonLib examples - don't ask me how it works... */
Eigen::Matrix<double, 3, 1> VisionSubsystem::GetEstimationStdDevs(frc::Pose2d pose) {
    Eigen::Matrix<double, 3, 1> stddevs = VisionConstants::kSingleTagStdDevs;

    auto targets = m_camera.GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt: targets) {
        auto tagPose = m_photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose) {
            numTags++;
            avgDist += tagPose->ToPose2d().Translation().Distance(pose.Translation());
        }
    }
    if (numTags == 0) return stddevs;

    avgDist /= numTags;

    if (numTags > 1) stddevs = VisionConstants::kMultiTagStdDevs;

    if (numTags == 1 && avgDist > 4_m) {
        double maxDouble = std::numeric_limits<double>::max();
        stddevs = (Eigen::MatrixXd(3, 1) << maxDouble, maxDouble, maxDouble).finished();
    } else {
        stddevs = stddevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }

    return stddevs;
}