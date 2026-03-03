#pragma once
//SparkMax
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
//Feedforward + Motion profiling
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
// PID Profile and Controller stuff
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/TimedRobot.h>
#include <frc/geometry/Translation2d.h>

#include <set>

enum Level {COLLECT, DEFAULT, L1, L2, L3};

struct PIDConstants {
  double kP, kI, kD;
};

//check if this struct is being used.
struct ArmFeedforwardConstants {
  units::volt_t kS, kG;
  units::unit_t<frc::ArmFeedforward::kv_unit> kV;
  units::unit_t<frc::ArmFeedforward::ka_unit> kA;
};

struct ElevatorFeedforwardConstants {
  units::volt_t kS, kG;
  units::unit_t<frc::ElevatorFeedforward::kv_unit> kV;
  units::unit_t<frc::ElevatorFeedforward::ka_unit> kA;
};

namespace General {
  inline constexpr double kFloatTolerance = 0.0001;
}

//FIXME NEED TO CHECK CAN IDs
namespace CAN_Constants {
  //Subsystem CAN IDs 
  inline constexpr int kElevatorMotorLeftCAN_ID = 21;
  inline constexpr int kElevatorMotorRightCAN_ID = 22;
  inline constexpr int kElevatorMotorSecondStageCAN_ID = 23;

  // not configurated yet
  inline constexpr int kElevatorArmMotorCAN_ID = 24;
}  

namespace ElevatorConstants {
  //PID Trapezoidal Controller
  inline constexpr units::second_t kDt = 20_ms;
  
  //Smart Current Limit
  inline constexpr int kCurrentLimit = 50;

  //First Stage Controller
  inline constexpr PIDConstants kFirstStagePID = {
    /* kP */ 0.2,
    /* kI */ 0.0,
    /* kD */ 0.0
  };
  inline constexpr ElevatorFeedforwardConstants kFirstStageFeedforward = {
    /* kS */ 0.11_V,
    /* kG */ 0.1_V,
    /* kV */ 19.5_V / 1_mps,
    /* kA */ 0.0_V / 1_mps_sq
  };
  
  //Second Stage Controller
  inline constexpr PIDConstants kSecondStagePID = {
    /* kP */ 0.2,
    /* kI */ 0.0,
    /* kD */ 0.0
  };
  inline constexpr ElevatorFeedforwardConstants kSecondStageFeedforward = {
    /* kS */ 0.225_V,
    /* kG */ 0.0_V,
    /* kV */ 28.0_V / 1_mps,
    /* kA */ 0.0_V / 1_mps_sq
  };

  inline constexpr double maxOutput = 1.0;

  //PID Profile
  inline constexpr frc::TrapezoidProfile<units::meter>::Constraints trapezoidProfileFirstStage{0.6_mps, 2.0_mps_sq};
  inline constexpr frc::TrapezoidProfile<units::meter>::Constraints trapezoidProfileSecondStage{0.3_mps, 1.0_mps_sq};

  //Elevator Goals
  inline constexpr units::meter_t eLevel0Goal = 0.0_m;
  inline constexpr units::meter_t eLevel1Goal = eLevel0Goal;
  inline constexpr units::meter_t eLevel2Goal = 0.3_m;
  inline constexpr units::meter_t eLevel3Goal = 0.6_m;
  inline constexpr units::meter_t eLevel4Goal = 0.8_m;

  inline const units::meter_t kFirstStageDefaultHeight = 0.3844355046749115_m;

  //Encoder Position
  inline constexpr units::meter_t resetEncoder = 0.0_m;

  // Elevator limits
  inline constexpr units::meter_t extendSoftLimitFirstStage = 0.68_m;
  inline constexpr units::meter_t extendSoftLimitSecondStage = 0.62_m;

  inline constexpr units::meter_t retractSoftLimitFirstStage = 0.0_m;
  inline constexpr units::meter_t retractSoftLimitSecondStage = 0.38873863697052_m;


  // Maximum Elevator Heights
  inline constexpr units::meter_t kMaxFirstStageHeight = extendSoftLimitFirstStage - 0.01_m;
  inline constexpr units::meter_t kMaxSecondStageHeight = extendSoftLimitSecondStage - 0.01_m;

  // Initial Elevator heights
  inline constexpr units::meter_t kInitFirstStageHeight = 0.0_m;
  inline constexpr units::meter_t kInitSecondStageHeight = 0.37900087237358093_m;

  // Limit Switch Locations
  inline constexpr units::meter_t kResetFirstStageHeight = 0.0_m;
  inline constexpr units::meter_t kResetSecondStageHeight = 0.0_m;
  //Elevator Height Conversion:
  /* DIAMETERS OF THE MOTOR SPROCKETS:
  55 mm - 1st stage
  38 mm - 2nd stage
  CIRCUMFERENCE OF THE MOTOR SPROCKETS:
  0.1727876 m - 1st stage
  0.1193805 m - 2nd stage
  DISTANCE PER TURN = CIRCUMFERENCE */
  inline constexpr units::meter_t distancePerTurnFirstStage = 0.1727876_m;
  inline constexpr units::meter_t distancePerTurnSecondStage = 0.1193805_m;

  //Gear Ratio
  inline constexpr double gearBoxGearRatio = 1.0 / 27.0;

  inline constexpr double gearRatioFirstStage = gearBoxGearRatio * distancePerTurnFirstStage.value();
  inline constexpr double gearRatioSecondStage = gearBoxGearRatio * distancePerTurnSecondStage.value();

  inline constexpr units::meter_t kElevatorPositionTolerance = 0.03_m;
  inline constexpr units::meters_per_second_t kElevatorVelocityTolerance = 0.1_mps;

  inline constexpr units::meter_t kElevatorMinHeightCollect = 1_m; //issue 70 - update this position
  inline constexpr units::meter_t kElevatorPlaceCoral = -0.1_m; // issue 70 - update this amount
  
  //Elevator DIO port
  inline constexpr int kFirstStageLimitSwitchPin = 1;
  inline constexpr int kSecondStageLimitSwitchPin = 2; // TODO

  inline constexpr units::meter_t kElevatorClearanceThreshold = kInitSecondStageHeight + 0.15_m;

  inline constexpr units::meters_per_second_t kManualMaxVelocity = 0.3_mps;
  inline constexpr units::meter_t kManualRetractLimitTolerance = 0.01_m;
  }

namespace ArmConstants {
  //PID Profile
  inline constexpr frc::TrapezoidProfile<units::turn>::Constraints trapezoidProfile{0.8_tps, 4.0_tr_per_s_sq};
  
  //PID Trapezoidal Controller
  inline constexpr units::second_t kDt = 20_ms;

  //First Stage PID Controller 
  inline constexpr double kP = 0.5;
  inline constexpr double kI = 0.0;
  inline constexpr double kD = 0.0;
  inline constexpr double maxOutput = 1.0;

  //Arm feedforward
  inline constexpr units::volt_t kS = 0.12_V;
  inline constexpr units::volt_t kG = 0.25_V;
  inline constexpr auto kV = 4.7_V / 1_tps;
  inline constexpr auto kA = 0.0_V / 1_tr_per_s_sq;

  // Arm limits
  inline constexpr units::turn_t extendSoftLimit = 0.23_tr;
  inline constexpr units::turn_t retractSoftLimit = -0.24_tr;

  //Arm Goals - this is the output of the gearbox (not the motor)
  inline constexpr units::turn_t aLevel0Goal = retractSoftLimit;
  inline constexpr units::turn_t aLevel1Goal = -0.15_tr;
  inline constexpr units::turn_t aLevel2Goal = -0.1_tr;
  inline constexpr units::turn_t aLevel3Goal = 0.0_tr;
  inline constexpr units::turn_t aLevel4Goal = 0.1_tr;

  inline constexpr double gearBoxGearRatio = 1.0 / 27.0;
  // this is the ratio between the motor sprocket teeth and the teeth on sprocket connected to the climb
  inline constexpr double motorSprocketRatio = 12.0 / 18.0;
  inline constexpr double gearRatio = gearBoxGearRatio * motorSprocketRatio;

  inline constexpr units::turn_t kArmPositionTolerance = 0.02_tr; // issue 70 - update this tolerance
  inline constexpr units::turns_per_second_t kArmVelocityTolerance = 0.1_tps;
  inline constexpr units::turn_t kArmPlaceCoral = -0.1_tr; // issue 70 - update this amount

  //Encoder Position
  inline constexpr units::turn_t resetEncoder = -0.23_tr; // with backlash

  //Arm DIO port
  inline constexpr int k_limitSwitchArmPin = 3;
  inline constexpr units::turn_t kArmClearanceThreshold = -0.17_tr; //ISSUE 112 - update this

  inline constexpr units::turns_per_second_t kManualMaxVelocity = 0.3_tps; // TODO: update this
}

struct ElevatorArmGoal {
  units::meter_t elevator;
  units::turn_t arm;
};

namespace ElevatorAndArmConstants {
  inline constexpr ElevatorArmGoal kDefaultGoal = {ElevatorConstants::kFirstStageDefaultHeight + ElevatorConstants::kMaxSecondStageHeight, ArmConstants::retractSoftLimit};
  inline constexpr ElevatorArmGoal kLevel1Goal = {0.21507354080677032_m + 0.0003047217323910445_m, 0.17623277008533478_tr};
  inline constexpr ElevatorArmGoal kLevel2Goal = {0.06033877283334732_m + 0.6304726004600525_m, 0.1756448596715927_tr};
  inline constexpr ElevatorArmGoal kLevel3Goal = {0.6771453022956848_m + 0.6319462656974792_m, 0.23_tr};
};

