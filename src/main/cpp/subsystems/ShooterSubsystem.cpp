#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"

#include <rev/config/SparkMaxConfig.h>

#include <units/math.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;

using namespace rev::spark;

ShooterSubsystem::ShooterSubsystem()
: m_motor(HardwareConstants::kShooterFlywheelID, "rio"),
  m_follower(HardwareConstants::kShooterFlywheelFollowerID, "rio")
  {
    TalonFXConfiguration flyWheelMotorConfig = createMotorConfig();
    m_motor.GetConfigurator().Apply(flyWheelMotorConfig);
    m_follower.GetConfigurator().Apply(flyWheelMotorConfig);

    m_follower.SetControl(Follower{m_motor.GetDeviceID(), false});

    //shooter angle motor
    rev::spark::SparkMaxConfig angleMotorConfig;

    angleMotorConfig
    .SmartCurrentLimit(ShooterConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kBrake)
    .Inverted(true);
    
    angleMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(ShooterConstants::angle::kP)
      .I(ShooterConstants::angle::kI)
      .D(ShooterConstants::angle::kD)
      .OutputRange(-1, 1);

    m_angleEncoder.SetPosition(0);

    angleMotorConfig.limitSwitch
    //.ForwardLimitSwitchEnabled(true)  // fixme - this looks to be depracated (can be removed), need to use forward limit switch behaviour
    .ForwardLimitSwitchTriggerBehavior(LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
    .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)  // fixme - check if lim switch is connected as N/O or N/C
    .ForwardLimitSwitchPosition(ShooterConstants::kMinAngleSoftLimit.value()); // fixme - check if this updates encoder position

    angleMotorConfig.softLimit
    .ForwardSoftLimit(ShooterConstants::kMinAngleSoftLimit.value()).ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(ShooterConstants::kMaxAngleSoftLimit.value()).ReverseSoftLimitEnabled(true);

    angleMotorConfig.encoder
    .PositionConversionFactor(ShooterConstants::kAngleGearRatio)
    .VelocityConversionFactor(ShooterConstants::kAngleGearRatio);

    SetDefaultCommand(ShootCommand(0_tps));

    m_angleMotor.Configure(
      angleMotorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    m_rotorVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/RotorVel").Publish();
    m_motorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/MotorWheelVel").Publish();
    m_followerMotorWheelVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/FollowerMotorWheelVel").Publish();
    m_motorCurrentPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/MotorCurrent").Publish();
    m_followerMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/FollowerMotorCurrent").Publish();
    m_shooterAnglePub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleOfShooter").Publish();
    m_angleMotorVoltagePub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleMotorVoltage").Publish();
    m_angleMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleMotorCurrent").Publish();
    m_angleMotorLimSwitchPub = nt::NetworkTableInstance::GetDefault()
        .GetBooleanTopic("Angle/LimitSwitch").Publish();
}

TalonFXConfiguration ShooterSubsystem::createMotorConfig(){
    TalonFXConfiguration motorConfig;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    motorConfig.Feedback.SensorToMechanismRatio = ShooterConstants::kgearRatio;
    motorConfig.Slot0.kP = ShooterConstants::motor::kP;
    motorConfig.Slot0.kI = ShooterConstants::motor::kI;
    motorConfig.Slot0.kD = ShooterConstants::motor::kD;
    motorConfig.Slot0.kV = ShooterConstants::motor::kV;
    motorConfig.Slot0.kS = ShooterConstants::motor::kS;
    motorConfig.Slot0.kA = ShooterConstants::motor::kA;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50_A; // fixme - these values will have to be changed
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60_A;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    motorConfig.MotorOutput.Inverted = true;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;

    return motorConfig;
}

void ShooterSubsystem::Periodic() {
        /* publish current state */
        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_follower.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
        m_shooterAnglePub.Set(m_angleEncoder.GetPosition());
        m_angleMotorVoltagePub.Set(m_angleMotor.GetAppliedOutput());
        m_angleMotorCurrentPub.Set(m_angleMotor.GetOutputCurrent());

        // Get angle motor limit switch status
        bool forwardLimitPressed = m_angleMotor.GetForwardLimitSwitch().Get();
        m_angleMotorLimSwitchPub.Set(forwardLimitPressed);
}

void ShooterSubsystem::Shoot(units::volt_t volts){
    m_motor.SetVoltage(volts);
}

void ShooterSubsystem::GoToAngle(units::turn_t angle) {
    m_angleController.SetReference(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
}

void ShooterSubsystem::ShootAngularVelocity(units::turns_per_second_t angularVelocity) {
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage(angularVelocity);
    m_motor.SetControl(velocityVoltage);
}

frc2::CommandPtr ShooterSubsystem::ShootCommand(units::turns_per_second_t angularVelocity) {
    return Run([this, angularVelocity]{
                ShootAngularVelocity(angularVelocity);
            });
}

units::meter_t ShooterSubsystem::DistanceToHub(frc::Pose2d robotPose){
    return CalculateDistanceBetweenPoints(robotPose.Translation(), FieldConstants::kHubPosition);
}

units::turns_per_second_t ShooterSubsystem::CalculateFlyWheelSpeed(meter_t distance, degree_t angle) {
    meters_per_second_t ballSpeed = CalculateBallSpeed(distance, angle);

    double metresPerTurn = 2 * std::numbers::pi * ShooterConstants::kFlyWheelRadius.value();
    return units::turns_per_second_t{ballSpeed.value()/metresPerTurn};
}

// derived from omnicalculator trajectory formula >> https://www.omnicalculator.com/physics/trajectory-projectile-motion
// done by rearranging the formula to find the speed for a given distance and angle 

meters_per_second_t ShooterSubsystem::CalculateBallSpeed(meter_t distance, degree_t angle) {
        auto cosine = cos(angle);
        auto tangent = tan(angle);

        meter_t adjustedHeight = FieldConstants::HubHeight - ShooterConstants::startHeight;

        meters_per_second_t speed = 
            sqrt(
                (FieldConstants::gravity * pow<2>(distance)) /
                (2 * pow<2>(cosine) * (distance * tangent - adjustedHeight))
            );

        return speed;
}

meter_t ShooterSubsystem::CalculateDistanceBetweenPoints(frc::Translation2d p1, frc::Translation2d p2) {
    return p1.Distance(p2);
}