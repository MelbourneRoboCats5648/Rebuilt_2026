#include <drive/DriveModule.h>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include <constants/HardwareConstants.h>

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::controls;
using namespace units::voltage;




DriveModule::DriveModule(
    int speedMotorID, int directionMotorID, int directionEncoderID,
    turn_t magOffset
) : m_speedMotor(speedMotorID, HardwareConstants::kPhoenixCAN),
    m_directionMotor(directionMotorID, HardwareConstants::kPhoenixCAN),
    m_directionEncoder(directionEncoderID, HardwareConstants::kPhoenixCAN)
{
    SetSpeedMotorConfig();
    SetDirectionMotorConfig();
    SetDirectionEncoderConfig(magOffset);

    m_directionController.EnableContinuousInput(-0.5_tr, 0.5_tr);
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
    directionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
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

void DriveModule::SetState(frc::SwerveModuleState state){
    // encoder range -0.5 + 0.5 
    units::angle::radian_t encoderCurrentAngleRadians = 
                            m_directionEncoder.GetAbsolutePosition().GetValue();
    
    state.Optimize(encoderCurrentAngleRadians);

    state.CosineScale(encoderCurrentAngleRadians);

    turns_per_second_t desiredWheelSpeed{(state.speed.value())/SpeedMotor::kWheelCircumference.value()};
    m_speedMotor.SetControl(VelocityVoltage{desiredWheelSpeed});

    double turnOutput = m_directionController.Calculate(
        encoderCurrentAngleRadians, state.angle.Radians());

    m_directionMotor.SetVoltage(volt_t{turnOutput});

}

void DriveModule::ResetModulePosition()
{
    m_speedMotor.SetPosition(turn_t{0.0});

}

units::meters_per_second_t DriveModule::GetSpeed(){
    return(m_speedMotor.GetVelocity().GetValueAsDouble() * SpeedMotor::kWheelCircumference.value()) * 1_mps;
}

frc::Rotation2d DriveModule::GetAngle() {
    units::radian_t turnAngle = m_directionEncoder.GetAbsolutePosition().GetValue();
    return turnAngle; 
}

frc::SwerveModuleState DriveModule::GetState() {
    return {GetSpeed(), GetAngle()};
}

frc::SwerveModulePosition DriveModule::GetPosition() {
    return {
        m_speedMotor.GetPosition().GetValueAsDouble() * SpeedMotor::kWheelCircumference,
        GetAngle()
    };
}