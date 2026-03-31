#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>

namespace VisionConstants {
    inline const frc::AprilTagFieldLayout kTagLayout{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};

    inline const std::string kCameraName = "photonvision";

    inline constexpr frc::Transform3d kCameraLocation{
        frc::Translation3d{-0.300_m, 0.0_m, 0.485_m},
        frc::Rotation3d{0_deg, 0_deg, 180_deg}
    };

    inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
    inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
};