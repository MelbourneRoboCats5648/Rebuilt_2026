#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>

class Feeder {

    public:
             Feeder();
        void ResetEncoder();
        void ResetMotor();
        frc2::CommandPtr FeedCommand(units::turns_per_second_t feedVelocity);
        void Feed(units::volt_t volts);


    private:
        rev::spark::SparkMax m_motor;
};
