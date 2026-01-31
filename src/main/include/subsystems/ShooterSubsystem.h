#pragma once

#include <frc2/command/SubsystemBase.h>
#include "units/velocity.h"
#include "units/angle.h"
#include "units/length.h"


using namespace units::velocity;
using namespace units::angle;
using namespace units::length;

class ShooterSubsystem : public frc2::SubsystemBase {

    public:
        ShooterSubsystem();
        meters_per_second_t CalculateShooterSpeed(meter_t distance, degree_t angle);

    private:





};