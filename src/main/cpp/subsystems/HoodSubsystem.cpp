#include "subsystems/HoodSubsystem.h"
#include "rev/config/SparkMaxConfig.h"

HoodSubsystem::HoodSubsystem(DriveSubsystem& drive):
m_drive(drive)
{
    rev::spark::SparkMaxConfig angleMotorConfig;
    
    m_angleMotor.Configure(
      angleMotorConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );

    angleMotorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .P(HoodConstants::kP)
      .I(HoodConstants::kI)
      .D(HoodConstants::kD)
      .OutputRange(-1, 1);

    m_angleEncoder.SetPosition(HoodConstants::kMaxAngleSoftLimit.value());

    angleMotorConfig.softLimit
    .ForwardSoftLimit(HoodConstants::kMaxAngleSoftLimit.value()).ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(HoodConstants::kMinAngleSoftLimit.value()).ReverseSoftLimitEnabled(true);

    angleMotorConfig.encoder
    .PositionConversionFactor(HoodConstants::kAngleDegreesPerTurn)
    .VelocityConversionFactor(HoodConstants::kAngleDegreesPerTurn);

    m_angleMotor.Configure(
      angleMotorConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );

    SetDefaultCommand(
        frc2::cmd::Either(
            Run([this] { GoToAngle(m_targetAngle); }), // if calibrated, run the hood motor (not the flywheel)
            RetractToLimitCommand(), // otherwise, do the hood calibration
            [this] {
                if (!m_isCalibrated) {
                    m_isCalibrated = true; // should be false only once
                    return false;
                } else {
                    return true;
                }
            }
        ).Repeatedly() // if calibration finishes, re-run this command (which will take us to default shoot command)
    );

    m_hoodAnglePub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Hood/AngleOfShooter").Publish();
    m_hoodAngleVelocityPub = nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Hood/AngleMotorVelocity").Publish();
    m_angleMotorVoltagePub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Hood/AngleMotorVoltage").Publish();
    m_angleMotorCurrentPub= nt::NetworkTableInstance::GetDefault()
        .GetDoubleTopic("Hood/AngleMotorCurrent").Publish();
}

void HoodSubsystem::Periodic() {
    m_hoodAnglePub.Set(m_angleEncoder.GetPosition());
    m_angleMotorVoltagePub.Set(m_angleMotor.GetAppliedOutput());
    m_angleMotorCurrentPub.Set(m_angleMotor.GetOutputCurrent());
    m_hoodAngleVelocityPub.Set(GetAngleVelocity().value());

    /* now being done in the shooter subsystem
    units::meter_t distanceToTarget = m_drive.DistanceToTarget();
    units::turn_t targetAngle = (distanceToTarget > HoodConstants::kRangeThreshold) ? HoodConstants::kMinAngle : HoodConstants::kMaxAngle;
    SetTargetAngle(targetAngle);
    */
}

void HoodSubsystem::GoToAngle(units::degree_t angle) {
    double clampedAngleDeg = std::clamp(angle.value(), HoodConstants::kMinAngle.value(), HoodConstants::kMaxAngle.value());
    m_angleController.SetSetpoint(clampedAngleDeg, rev::spark::SparkLowLevel::ControlType::kPosition);
}

frc2::CommandPtr HoodSubsystem::GoToAngleCommand(units::degree_t angle) {
    return RunOnce([this, angle] { GoToAngle(angle); });
}

frc2::CommandPtr HoodSubsystem::GoToAngleCommand() {
    return RunOnce([this] { GoToAngle(m_targetAngle); });
}

void HoodSubsystem::SetTargetAngle(units::turn_t angle)
{
    m_targetAngle = angle;
}

frc2::CommandPtr HoodSubsystem::SetTargetAngleCommand(units::degree_t angle) {
    return RunOnce([this, angle]{
                SetTargetAngle(angle);
            });
}

degrees_per_second_t HoodSubsystem::GetAngleVelocity() {
    return degrees_per_second_t{m_angleEncoder.GetVelocity() / 60}; // convert from RPM to turns per second
}

frc2::CommandPtr HoodSubsystem::RetractToLimitCommand() {
    return
        /* extend for a bit to ensure we'll be able to retract the hood over a distance */
        RunOnce([this] {
            // set position to max angle to defeat soft limit
            m_angleEncoder.SetPosition(HoodConstants::kMaxAngleSoftLimit.value());
            // then extend the hood
            m_angleMotor.SetVoltage(-HoodConstants::kCalibrationVoltage);
        })
        .AndThen(frc2::cmd::Wait(HoodConstants::kCalibrationPreTime))
        .AndThen(
            RunOnce([this] {
                // initially set encoder position to min angle
                m_angleEncoder.SetPosition(HoodConstants::kMinAngleSoftLimit.value());
                // then pull the hood in at a slow speed
                m_angleMotor.SetVoltage(HoodConstants::kCalibrationVoltage);
            })
            .AndThen(frc2::cmd::Sequence(
                // 1. we speed up past the threshold
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() > HoodConstants::kCalibrationVelocityThreshold; }),
                // 2. then we slow down past it again - indicating that we've reached the end
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() < HoodConstants::kCalibrationVelocityThreshold; })
            ))
            .WithTimeout(HoodConstants::kCalibrationTimeout)
            .FinallyDo([this] {
                m_angleMotor.StopMotor();
                m_angleEncoder.SetPosition(HoodConstants::kMaxAngleSoftLimit.value());
            })
        );
}

frc2::CommandPtr HoodSubsystem::ExtendToLimitCommand() {
    return
        /* retract for a bit to ensure we'll be able to extend the hood over a distance */
        RunOnce([this] {
            // set position to min angle to defeat soft limit
            m_angleEncoder.SetPosition(HoodConstants::kMinAngleSoftLimit.value());
            // then retract the hood
            m_angleMotor.SetVoltage(HoodConstants::kCalibrationVoltage);
        })
        .AndThen(frc2::cmd::Wait(HoodConstants::kCalibrationPreTime))
        .AndThen(
            RunOnce([this] {
                // initially set encoder position to max angle
                m_angleEncoder.SetPosition(HoodConstants::kMaxAngleSoftLimit.value());
                // then pull the hood out at a slow speed
                m_angleMotor.SetVoltage(-HoodConstants::kCalibrationVoltage);
            })
            .AndThen(frc2::cmd::Sequence(
                // 1. we speed up past the threshold
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() < -HoodConstants::kCalibrationVelocityThreshold; }),
                // 2. then we slow down past it again - indicating that we've reached the end
                frc2::cmd::WaitUntil([this] { return GetAngleVelocity() > -HoodConstants::kCalibrationVelocityThreshold; })
            ))
            .WithTimeout(HoodConstants::kCalibrationTimeout)
            .FinallyDo([this] {
                m_angleMotor.StopMotor();
                m_angleEncoder.SetPosition(HoodConstants::kMinAngleSoftLimit.value());
            })
        );
}
