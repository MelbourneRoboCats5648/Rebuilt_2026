#include <subsystems/FeederSubsystem.h>

#include <rev/config/SparkMaxConfig.h>

#include <constants/ShooterConstants.h>
#include <constants/HardwareConstants.h>

FeederSubsystem::FeederSubsystem()
    : m_motor(HardwareConstants::kShooterFeederID, rev::spark::SparkMax::MotorType::kBrushless){
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
    .SmartCurrentLimit(ShooterConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
    .Inverted(true);
    
    m_motor.Configure(
      motorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );
}

void FeederSubsystem::Feed() {
    m_motor.SetVoltage(ShooterConstants::kFeederVoltage);
}

void FeederSubsystem::Stop() {
    m_motor.StopMotor();
}

frc2::CommandPtr FeederSubsystem::FeedCommand() {
    return Run([this] { Feed(); })
        .FinallyDo([this] { Stop(); });
}