#include "subsystems/ShooterSubsystem.h"
#include "constants/ShooterConstants.h"
#include "constants/FieldConstants.h"

#include <rev/config/SparkMaxConfig.h>

#include <units/math.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>

using namespace units::math;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::signals;

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
    .SetIdleMode(rev::spark::SparkMaxConfig::kBrake);
    // .Inverted(true);
    
    angleMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(ShooterConstants::angle::kP)
      .I(ShooterConstants::angle::kI)
      .D(ShooterConstants::angle::kD)
      .OutputRange(-1, 1);

    m_angleEncoder.SetPosition(ShooterConstants::kMaxAngleSoftLimit.value());

    angleMotorConfig.softLimit
    .ForwardSoftLimit(ShooterConstants::kMaxAngleSoftLimit.value()).ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(ShooterConstants::kMinAngleSoftLimit.value()).ReverseSoftLimitEnabled(true);

    angleMotorConfig.encoder
    .PositionConversionFactor(ShooterConstants::kAngleDegreesPerTurn)
    .VelocityConversionFactor(ShooterConstants::kAngleDegreesPerTurn);

    //SetDefaultCommand(ShootCommand(0_tps));
    SetDefaultCommand(DefaultShootCommand());

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
    m_shooterAngleVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleMotorVelocity").Publish();
    m_angleMotorVoltagePub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleMotorVoltage").Publish();
    m_angleMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Angle/AngleMotorCurrent").Publish();

    m_shooterVoltagePub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/Voltage").Publish();
    m_shooterTargetVelPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/TargetVelocity").Publish();

    m_adjustedSpeedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/adjustedSpeed").Publish();
    m_requiredSpedPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Shooter/requiredSpeed").Publish();

    // uncomment below to allow smart dashboard to display 'ShooterVelocity' string
    // frc::SmartDashboard::PutNumber("ShooterVelocity", 0.0); // initialise shooter target velocity with default of 0.0
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

        // uncomment below to allow target velocity to be set via smart dashboard
        //m_targetVelocity = units::turns_per_second_t{frc::SmartDashboard::GetNumber("ShooterVelocity", 0.0)};

        m_rotorVelPub.Set(m_motor.GetRotorVelocity().GetValueAsDouble());
        m_motorWheelVelPub.Set(m_motor.GetVelocity().GetValueAsDouble());
        m_followerMotorWheelVelPub.Set(m_follower.GetVelocity().GetValueAsDouble());
        m_motorCurrentPub.Set(m_motor.GetTorqueCurrent().GetValueAsDouble());
        m_followerMotorCurrentPub.Set(m_follower.GetTorqueCurrent().GetValueAsDouble());
        m_shooterAnglePub.Set(m_angleEncoder.GetPosition());
        m_angleMotorVoltagePub.Set(m_angleMotor.GetAppliedOutput());
        m_angleMotorCurrentPub.Set(m_angleMotor.GetOutputCurrent());
        m_shooterAngleVelocityPub.Set(GetAngleVelocity().value());
        m_shooterVoltagePub.Set(m_motor.GetMotorVoltage().GetValueAsDouble());

        m_requiredSpedPub.Set(m_requiredSpeed);
        m_adjustedSpeedPub.Set(m_adjustedSpeed);


}

void ShooterSubsystem::Shoot(units::volt_t volts){
    m_motor.SetVoltage(volts);
}

void ShooterSubsystem::GoToAngle(units::degree_t angle) {
    double clampedAngleDeg = std::clamp(angle.value(), ShooterConstants::kMinAngle.value(), ShooterConstants::kMaxAngle.value());
    m_angleController.SetReference(clampedAngleDeg, rev::spark::SparkLowLevel::ControlType::kPosition);
}

frc2::CommandPtr ShooterSubsystem::GoToAngleCommand(units::degree_t angle) {
    return RunOnce([this, angle] { GoToAngle(angle); });
}

void ShooterSubsystem::ShootAngularVelocity(units::turns_per_second_t angularVelocity) {
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage(angularVelocity);
    m_motor.SetControl(velocityVoltage);
    m_shooterTargetVelPub.Set(angularVelocity.value());
}

void ShooterSubsystem::SetTargetVelocity(units::turns_per_second_t velocity){
    m_targetVelocity = velocity;
}

units::turns_per_second_t ShooterSubsystem::GetTargetVelocity() const{
    return m_targetVelocity;
}

frc2::CommandPtr ShooterSubsystem::DefaultShootCommand() {
    return Run([this]{
                ShootAngularVelocity(m_targetVelocity);
            });
}

// This command will be used to set the velocity of the flywheel shooter along with the default shoot command
frc2::CommandPtr ShooterSubsystem::SetTargetVelocityCommand(units::turns_per_second_t angularVelocity) {
    return RunOnce([this, angularVelocity]{
                SetTargetVelocity(angularVelocity);
            });
}

units::meter_t ShooterSubsystem::DistanceToHub(frc::Pose2d robotPose){
    frc::Translation2d hubPosition =
        (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue)
            ? FieldConstants::kBlueHubPosition
            : FieldConstants::kRedHubPosition;
    return CalculateDistanceBetweenPoints(robotPose.Translation(), hubPosition);
}

units::turns_per_second_t ShooterSubsystem::CalculateFlyWheelSpeed(meter_t distance, degree_t angle) {
    meters_per_second_t requiredSpeed = CalculateBallSpeed(distance, angle);
    meters_per_second_t adjustedSpeed = AdjustedBallSpeed(requiredSpeed);

    m_requiredSpeed = requiredSpeed.value();
    m_adjustedSpeed = adjustedSpeed.value();

    double metresPerTurn = 2 * std::numbers::pi * ShooterConstants::kFlyWheelRadius.value();
    return units::turns_per_second_t{adjustedSpeed.value() / metresPerTurn};
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

meters_per_second_t ShooterSubsystem::AdjustedBallSpeed(meters_per_second_t actualSpeed) {
    // coefficients found from curve fitting
    double a = -0.8418;
    double b = 14.6320;
    double c = -46.5525;

    double x = actualSpeed.value();

    meters_per_second_t adjustedSpeed =  meters_per_second_t(a * x * x + b * x + c);
    return adjustedSpeed;
}


meter_t ShooterSubsystem::CalculateDistanceBetweenPoints(frc::Translation2d p1, frc::Translation2d p2) {
    return p1.Distance(p2);
}

degrees_per_second_t ShooterSubsystem::GetAngleVelocity() {
    return degrees_per_second_t{m_angleEncoder.GetVelocity() / 60}; // convert from RPM to turns per second
}

frc2::CommandPtr ShooterSubsystem::RetractToLimitCommand() {
    return
        /* extend for a bit to ensure we'll be able to retract the hood over a distance */
        RunOnce([this] {
            // set position to max angle to defeat soft limit
            m_angleEncoder.SetPosition(ShooterConstants::kMaxAngleSoftLimit.value());
            // then extend the hood
            m_angleMotor.SetVoltage(-ShooterConstants::angle::kCalibrationVoltage);
        })
        .AndThen(frc2::cmd::Wait(ShooterConstants::angle::kCalibrationPreTime))
        .AndThen(
            RunOnce([this] {
                // initially set encoder position to min angle
                m_angleEncoder.SetPosition(ShooterConstants::kMinAngleSoftLimit.value());
                // then pull the hood in at a slow speed
                m_angleMotor.SetVoltage(ShooterConstants::angle::kCalibrationVoltage);
            })
            .AndThen(frc2::cmd::Sequence(
                // 1. we speed up past the threshold
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() > ShooterConstants::angle::kCalibrationVelocityThreshold; }),
                // 2. then we slow down past it again - indicating that we've reached the end
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() < ShooterConstants::angle::kCalibrationVelocityThreshold; })
            ))
            .WithTimeout(ShooterConstants::angle::kCalibrationTimeout)
            .FinallyDo([this] {
                m_angleMotor.StopMotor();
                m_angleEncoder.SetPosition(ShooterConstants::kMaxAngleSoftLimit.value());
            })
        );
}

frc2::CommandPtr ShooterSubsystem::ExtendToLimitCommand() {
    return
        /* retract for a bit to ensure we'll be able to extend the hood over a distance */
        RunOnce([this] {
            // set position to min angle to defeat soft limit
            m_angleEncoder.SetPosition(ShooterConstants::kMinAngleSoftLimit.value());
            // then retract the hood
            m_angleMotor.SetVoltage(ShooterConstants::angle::kCalibrationVoltage);
        })
        .AndThen(frc2::cmd::Wait(ShooterConstants::angle::kCalibrationPreTime))
        .AndThen(
            RunOnce([this] {
                // initially set encoder position to max angle
                m_angleEncoder.SetPosition(ShooterConstants::kMaxAngleSoftLimit.value());
                // then pull the hood out at a slow speed
                m_angleMotor.SetVoltage(-ShooterConstants::angle::kCalibrationVoltage);
            })
            .AndThen(frc2::cmd::Sequence(
                // 1. we speed up past the threshold
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() < -ShooterConstants::angle::kCalibrationVelocityThreshold; }),
                // 2. then we slow down past it again - indicating that we've reached the end
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() > -ShooterConstants::angle::kCalibrationVelocityThreshold; })
            ))
            .WithTimeout(ShooterConstants::angle::kCalibrationTimeout)
            .FinallyDo([this] {
                m_angleMotor.StopMotor();
                m_angleEncoder.SetPosition(ShooterConstants::kMinAngleSoftLimit.value());
            })
        );
}
