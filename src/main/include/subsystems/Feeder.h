#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>

class Feeder {

    public:
             Feeder();
        void ResetEncoder();
        void ResetMotor();


    private:
        rev::spark::SparkMax m_motor;
};
