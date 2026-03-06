#include <subsystems/IntakeSubsystem.h>
#include <constants/HardwareConstants.h>


IntakeSubsystem::IntakeSubsystem() :
m_ExtendRetractMotor(HardwareConstants::kExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_followerExtendRetractMotor(HardwareConstants::kFollowerExtendRetractMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_intakeMotor(HardwareConstants::kIntakeMotorID, rev::spark::SparkMax::MotorType::kBrushless)

{




}

turns_per_second_t IntakeSubsystem::CalculateIntakeSpeed(meters_per_second_t forwardRobotSpeed)
{
    // calculating the required speed of the wheel surface (tangent) with respect to the ground
    meters_per_second_t adjustedIntakeSurfaceSpeed = IntakeConstants::kTargetIntakeSurfaceSpeed + forwardRobotSpeed;

    // calculate angular velocity of intake wheel from the desired surface tangential speed
    meter_t distancePerTurn = IntakeConstants::PI * IntakeConstants::kIntakeWheelDiameter;

    turns_per_second_t requiredIntakeWheelSpeed = turns_per_second_t{adjustedIntakeSurfaceSpeed.value() / distancePerTurn.value()};

    return requiredIntakeWheelSpeed;
}