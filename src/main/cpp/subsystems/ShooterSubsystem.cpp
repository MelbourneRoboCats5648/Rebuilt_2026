#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"
#include <units/math.h>

using namespace units::math;

ShooterSubsystem::ShooterSubsystem(){
}


// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 

meters_per_second_t ShooterSubsystem::CalculateShooterSpeed(meter_t distance, degree_t angle) {
        auto cosine = cos(ShooterConstants::angle);
        auto tangent = tan(ShooterConstants::angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * pow<2>(ShooterConstants::distance)) /
                (2 * pow<2>(cosine) * (ShooterConstants::distance * tangent - adjustedHeight))
            );

        return speed;
}

