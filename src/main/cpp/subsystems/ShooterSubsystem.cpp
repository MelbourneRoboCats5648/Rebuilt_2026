#include <subsystems/ShooterSubsystem.h>

ShooterSubsystem::ShooterSubsystem(FlyWheelSubsystem& flyWheel, HoodSubsystem& hood, FeederSubsystem& feeder)
: m_flyWheel(flyWheel),
  m_hood(hood),
  m_feeder(feeder) 
{


}