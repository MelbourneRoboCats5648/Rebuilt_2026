#include "subsystems/ShooterSubsystem.h"
#include <units/math.h>

using namespace units;

ShooterSubsystem::ShooterSubsystem(){
    
}

units::radian_t ShooterSubsystem::GetShootingAngle(units::meter_t distance, units::meters_per_second_t velocity){
    //return 0_rad;
    units::radian_t shootingAngle;
//Write equation-atan doesnt work at the moment
//atan(xDist - sqrt(Pow(xDist))

    shootingAngle = units::math::atan(0.0);

   const double x_squared = math::pow<2, meter_t>(ShooterConstants::kXDist);
   const double v_squared = math::pow<2,meters_per_second_t(ShooterConstants::kShooterVelocity);
   const double finalHeight = ShooterConstants::kYDist - ShooterConstants::kInitialHeight;

    atan ((ShooterConstants::kXDist + sqrt(x_squared (- 2*x_squared* ShooterConstants::kAccelGravity* (ShooterConstants::kYDist - ShooterConstants::kInitialHeight))/ v_squared))/(ShooterConstants::kAccelGravity* x_squared/ v_squared)) 

}




