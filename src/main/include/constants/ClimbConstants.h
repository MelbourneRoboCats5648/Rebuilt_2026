#pragma once

#include <units/length.h>
#include <units/current.h>

using namespace units::length;
using namespace units::current;

namespace ClimbConstants {
    meter_t kInitHeight = (67_m); 
    meter_t kDefaultHeight = (67_m);

    meter_t kExtendSoftLimit = (67_m);
    meter_t kRetractSoftLimit = (67_m);

    constexpr int kCurrentLimit = (50);

    double kGearRatio = (67);

    

}



