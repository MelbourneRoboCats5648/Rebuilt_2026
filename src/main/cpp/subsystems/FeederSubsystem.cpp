#include <subsystems/FeederSubsystem.h>

#include <rev/config/SparkMaxConfig.h>

#include <constants/ShooterConstants.h>
#include <constants/HardwareConstants.h>

FeederSubsystem::FeederSubsystem()
    : m_motor(HardwareConstants::kShooterFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_leaderSideMotor(HardwareConstants::kShooterLeaderSideFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_followerSideMotor(HardwareConstants::kShooterFollowerSideFeederID, rev::spark::SparkMax::MotorType::kBrushless)
    {
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

    rev::spark::SparkMaxConfig leaderSideConfig;
    leaderSideConfig
        .SmartCurrentLimit(ShooterConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
    m_leaderSideMotor.Configure(
        leaderSideConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kPersistParameters
    );

    rev::spark::SparkMaxConfig followerSideConfig;
    followerSideConfig
        .SmartCurrentLimit(ShooterConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
        .Follow(m_leaderSideMotor, true); // inverted from leader
    m_followerSideMotor.Configure(
        followerSideConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kPersistParameters
    );
}

void FeederSubsystem::Feed() {
    m_motor.SetVoltage(ShooterConstants::kFeederVoltage);
    m_leaderSideMotor.SetVoltage(ShooterConstants::kSideFeederVoltage);
}

void FeederSubsystem::Stop() {
    m_motor.StopMotor();
    m_leaderSideMotor.StopMotor();
}

frc2::CommandPtr FeederSubsystem::FeedCommand() {
    return Run([this] { Feed(); })
        .FinallyDo([this] { Stop(); });
}