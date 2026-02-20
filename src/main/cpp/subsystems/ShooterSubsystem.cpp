#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"

#include <units/math.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;

ShooterSubsystem::ShooterSubsystem(int motorID, int followerID)
: m_motor(motorID, "rio"),
  m_follower(followerID, "rio")
{
    TalonFXConfiguration motorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(motorConfig);
    m_follower.GetConfigurator().Apply(motorConfig);

    m_follower.SetControl(Follower{m_motor.GetDeviceID(), false});

    SetDefaultCommand(ShootCommand(0_V));

};

TalonFXConfiguration ShooterSubsystem::createMotorConfig(){
    TalonFXConfiguration motorConfig;
    motorConfig.Slot0.kP = ShooterConstants::motor::kP;
    motorConfig.Slot0.kI = ShooterConstants::motor::kI;
    motorConfig.Slot0.kD = ShooterConstants::motor::kD;

    return motorConfig;
    
};

frc2::CommandPtr ShooterSubsystem::ShootCommand(units::volt_t volts) {
    return Run([this, volts]{
                m_motor.SetVoltage(volts);
            });
};

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
};