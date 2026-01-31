#pragma once
#include <frc2/command/SubsystemBase.h>
#include <constants/ShooterConstants.h>


class ShooterSubsystem : public frc2::SubsystemBase {

public:
    ShooterSubsystem();

 units::radian_t GetShootingAngle(units::meter_t distance, units::meters_per_second_t velocity);





private:


};