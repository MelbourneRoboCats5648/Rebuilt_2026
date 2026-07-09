#include <subsystems/FeederSubsystem.h>

#include <rev/config/SparkMaxConfig.h>

#include <constants/FeederConstants.h>
#include <constants/HardwareConstants.h>

FeederSubsystem::FeederSubsystem()
    : m_motor(HardwareConstants::kShooterFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_leftSideMotor(HardwareConstants::kShooterLeftSideFeederID, rev::spark::SparkMax::MotorType::kBrushless),
      m_rightSideMotor(HardwareConstants::kShooterRightSideFeederID, rev::spark::SparkMax::MotorType::kBrushless)
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

    rev::spark::SparkMaxConfig leftSideConfig;
    leftSideConfig
        .SmartCurrentLimit(FeederConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
        .Inverted(true);

    leftSideConfig.encoder
    .PositionConversionFactor(1.0 / FeederConstants::kSideMotorGearRatio)
    .VelocityConversionFactor((1.0 / FeederConstants::kSideMotorGearRatio) / 60.0); //might not be necessary 

    m_leftSideMotor.Configure(
        leftSideConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    rev::spark::SparkMaxConfig rightSideConfig;

    rightSideConfig
        .SmartCurrentLimit(FeederConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
        // .Follow(m_leftSideMotor, true); // inverted from leader
    
    rightSideConfig.encoder
    .PositionConversionFactor(1.0 / FeederConstants::kSideMotorGearRatio)
    .VelocityConversionFactor(1.0 / FeederConstants::kSideMotorGearRatio / 60.0);

    m_rightSideMotor.Configure(
        rightSideConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void FeederSubsystem::Feed() {
    m_motor.SetVoltage(FeederConstants::kFeederVoltage);
    m_leftSideMotor.SetVoltage(FeederConstants::kSideFeederVoltage + m_sideFeederVoltageDifference);
    m_rightSideMotor.SetVoltage(FeederConstants::kSideFeederVoltage - m_sideFeederVoltageDifference);
}

void FeederSubsystem::ReverseFeed() {
    m_motor.SetVoltage(-FeederConstants::kFeederVoltage);
    m_leftSideMotor.SetVoltage(-FeederConstants::kSideFeederVoltage);
    m_rightSideMotor.SetVoltage(-FeederConstants::kSideFeederVoltage);
}

void FeederSubsystem::Stop() {
    m_motor.StopMotor();
    m_leftSideMotor.StopMotor();
    m_rightSideMotor.StopMotor();
}

frc2::CommandPtr FeederSubsystem::FeedCommand() {
    return Run([this] { Feed(); })
        .FinallyDo([this] { Stop(); });
}

frc2::CommandPtr FeederSubsystem::ReverseFeedCommand() {
    return Run([this] { ReverseFeed(); })
        .FinallyDo([this] { Stop(); });
}

bool FeederSubsystem::IsStalling(){
    return m_motor.GetOutputCurrent() > 15.0; // fixme 
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