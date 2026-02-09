#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"
#include <units/math.h>

using namespace units::math;
using namespace units::angle;

ShooterSubsystem::ShooterSubsystem(){
}


// both have been derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed/angle for a given distance and speed/angle (respectively)


// speed equation 

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



// angle equation

radian_t ShooterSubsystem::GetShootingAngle(
    units::meter_t distance, 
    units::meters_per_second_t velocity){

auto xDistSquared = 
    pow<2>(ShooterConstants::XDist); 

auto velocitySquared = 
    ShooterConstants::ShooterVelocity * 
    ShooterConstants::ShooterVelocity; 

auto velocitySquaredSquared =
    velocitySquared * velocitySquared;

//the actual function starts here 

    radian_t angle = units::math::atan(
        (velocitySquared + units::math::sqrt(
        velocitySquaredSquared - 
        FieldConstants::gravity *
        (FieldConstants::gravity * xDistSquared + 
            2 * (ShooterConstants::YDist - ShooterConstants::startHeight) * velocitySquared))
            )
            /
        (FieldConstants::gravity * ShooterConstants::XDist));

        return angle;
}
