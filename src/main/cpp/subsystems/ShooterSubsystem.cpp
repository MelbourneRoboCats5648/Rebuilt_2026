#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"
#include <units/math.h>

using namespace units::math;
using namespace units::angle;

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


radian_t ShooterSubsystem::GetShootingAngle(units::meter_t distance, units::meters_per_second_t velocity){


//atan(ShooterConstants::XDist - sqrt(pow<2>(ShooterConstants::XDist)));          i dont understand what this is here for 

auto xDistSquared = ShooterConstants::XDist * ShooterConstants::XDist; // had to do it the crude way, litreally a*a = a^2. pow isnt working because units arent cancelling down
auto velocitySquared = ShooterConstants::ShooterVelocity * ShooterConstants::ShooterVelocity; 

meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;
 
radian_t angle = units::math::atan
(
((ShooterConstants::XDist + units::math::sqrt(xDistSquared * (- 2* xDistSquared * FieldConstants::gravity * 
(ShooterConstants::YDist - ShooterConstants::startHeight))/ velocitySquared))

/

(FieldConstants::gravity * xDistSquared / velocitySquared))

);

return angle; 
 
}
 
