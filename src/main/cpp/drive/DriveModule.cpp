#include <drive/DriveModule.h>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;




DriveModule::DriveModule(
    int speedMotorID, int directionMotorID, int directionEncoderID,
    turn_t magOffset
) : m_speedMotor(speedMotorID, "rio"),
    m_directionMotor(directionMotorID, "rio"),
    m_directionEncoder(directionEncoderID, "rio")
{
    SetSpeedMotorConfig();
    SetDirectionMotorConfig();
    SetDirectionEncoderConfig(magOffset);

}

void DriveModule::SetDirectionEncoderConfig(turn_t magOffset){
    CANcoderConfiguration cancoderConfig; 
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = magOffset;

    m_directionEncoder.GetConfigurator().Apply(cancoderConfig);

}

void DriveModule::SetDirectionMotorConfig(){
    TalonFXConfiguration directionMotorConfig;
    directionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    directionMotorConfig.CurrentLimits.SupplyCurrentLimit = DirectionMotor::kMaxCurrent;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = DirectionMotor::kLowerCurrentLimit;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerTime = DirectionMotor::kLowerLimitTime;
    directionMotorConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwis
    directionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake; // fixme - check if this should be Brake or Coast
    m_directionMotor.GetConfigurator().Apply(directionMotorConfig);
}

void DriveModule::SetSpeedMotorConfig(){
    TalonFXConfiguration speedMotorConfig;
    speedMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    speedMotorConfig.Feedback.SensorToMechanismRatio = SpeedMotor::kGearRatio;
    speedMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    speedMotorConfig.Slot0.kP = SpeedMotor::kP;
    speedMotorConfig.Slot0.kI = SpeedMotor::kI;
    speedMotorConfig.Slot0.kD = SpeedMotor::kD;
    speedMotorConfig.Slot0.kV = SpeedMotor::kV;
    speedMotorConfig.Slot0.kS = SpeedMotor::kS;
    speedMotorConfig.Slot0.kA = SpeedMotor::kA;
    speedMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    speedMotorConfig.CurrentLimits.SupplyCurrentLimit = SpeedMotor::kMaxCurrent;
    speedMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = SpeedMotor::kLowerCurrentLimit;
    speedMotorConfig.CurrentLimits.SupplyCurrentLowerTime = SpeedMotor::kLowerLimitTime;
    speedMotorConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwise
    speedMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    
    m_speedMotor.GetConfigurator().Apply(speedMotorConfig);
}

void DriveModule::StopMotors()
{
    m_directionMotor.Set(0);
    m_speedMotor.Set(0);
}