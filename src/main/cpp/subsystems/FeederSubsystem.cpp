#include <subsystems/FeederSubsystem.h>

#include <rev/config/SparkMaxConfig.h>

#include <constants/FlyWheelConstants.h>
#include <constants/FeederConstants.h>
#include <constants/HardwareConstants.h>

FeederSubsystem::FeederSubsystem()
    : m_motor(HardwareConstants::kShooterFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_leaderSideMotor(HardwareConstants::kShooterLeaderSideFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_followerSideMotor(HardwareConstants::kShooterFollowerSideFeederID, rev::spark::SparkMax::MotorType::kBrushless)
    {
    rev::spark::SparkMaxConfig motorConfig;

    motorConfig
    .SmartCurrentLimit(FeederConstants::kCurrentLimit)
    .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
    .Inverted(true);
    
    m_motor.Configure(
      motorConfig,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters
    );

    rev::spark::SparkMaxConfig leaderSideConfig;
    leaderSideConfig
        .SmartCurrentLimit(FeederConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
        .Inverted(true);
    m_leaderSideMotor.Configure(
        leaderSideConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    rev::spark::SparkMaxConfig followerSideConfig;
    followerSideConfig
        .SmartCurrentLimit(FeederConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
        // .Follow(m_leaderSideMotor, true); // inverted from leader
    m_followerSideMotor.Configure(
        followerSideConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void FeederSubsystem::Feed() {
    m_motor.SetVoltage(FeederConstants::kFeederVoltage);
    m_leaderSideMotor.SetVoltage(FeederConstants::kSideFeederVoltage + m_sideFeederVoltageDifference);
    m_followerSideMotor.SetVoltage(FeederConstants::kSideFeederVoltage - m_sideFeederVoltageDifference);
}

void FeederSubsystem::Stop() {
    m_motor.StopMotor();
    m_leaderSideMotor.StopMotor();
    m_followerSideMotor.StopMotor();
}

frc2::CommandPtr FeederSubsystem::FeedCommand() {
    return Run([this] { Feed(); })
        .FinallyDo([this] { Stop(); });
}

bool FeederSubsystem::IsStalling(){
     return m_motor.GetOutputCurrent() > 20.0;
}

frc2::CommandPtr FeederSubsystem::IncreaseFeederVoltageDifference() {
    return RunOnce([this] {
        m_sideFeederVoltageDifference += FeederConstants::kSideFeederVoltageDifferenceIncrement;
    });
}

frc2::CommandPtr FeederSubsystem::DecreaseFeederVoltageDifference() {
    return RunOnce([this] {
        m_sideFeederVoltageDifference -= FeederConstants::kSideFeederVoltageDifferenceIncrement;
    });
}