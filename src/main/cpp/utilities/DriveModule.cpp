#include <utilities/DriveModule.h>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>

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
    directionMotorConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30_A;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
     directionMotorConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwis
    directionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake; // fixme - check if this should be Brake or Coast
    m_directionMotor.GetConfigurator().Apply(directionMotorConfig);

}

void DriveModule::SetSpeedMotorConfig(){


    
}