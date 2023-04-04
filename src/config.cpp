

#include "main.h"

MotorGroup leftMotors({LEFT_BACK, LEFT_FRONT});
MotorGroup rightMotors({-RIGHT_BACK, -RIGHT_FRONT});

auto chassis = ChassisControllerBuilder() // chassis controller integrated
    .withMotors(leftMotors, rightMotors)
    .withDimensions(okapi::AbstractMotor::gearset::blue,{{6_in, 13_in}, imev5BlueTPR})
    .build();

// use for straight drives to correct for anomalies better than motion profiles will
// std::shared_ptr<ChassisControllerPID> chassisPID = std::make_shared<ChassisControllerPID>(
//     TimeUtilFactory::create(),
//     std::make_shared<SkidSteerModel>(std::make_shared<MotorGroup>(leftMotors), std::make_shared<MotorGroup>(rightMotors), 200),
//     std::make_unique<IterativePosPIDController>(0.0, 0.0, 0.0, 0.0, TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()), // distance
//     std::make_unique<IterativePosPIDController>(0.0, 0.0, 0.0, 0.0, TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()), // angle persist
//     std::make_unique<IterativePosPIDController>(0.0, 0.0, 0.0, 0.0, TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()), // turn
//     AbstractMotor::gearset::green,
//     chassis.getChassisScales()
// );
// TODO implement summing integral over a window
// REMEMBER TO chassisPID.startThread()

CustomAMPController motionProfile(
    
    1,1.09, 4.0, 10.0, // maxvel, accel, max jerk
    chassis.getChassisModel(),
    chassis.getChassisScales(),
    AbstractMotor::gearset::blue);

pathfollowing::AdaptivePurePursuit appController(
    std::make_unique<IterativePosPIDController>(0.1, 0.0, 0.0, 0.0, okapi::TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()),
    std::make_unique<IterativePosPIDController>(0.5, 0.0, 0.01, 0.0, okapi::TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()),
    200, 13.0); // turn was 0.6 // was 10

Motor mtrRB(3,false,okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
Motor mtrLB(4,false,okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
Motor mtrRF(5,false,okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
Motor mtrRB(6,false,okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);


void setAllMotorsBrakeMode(okapi::AbstractMotor::brakeMode mode)
{
    mtrRB.setBrakeMode(mode);
    mtrLB.setBrakeMode(mode);
    mtrRF.setBrakeMode(mode);
    mtrLF.setBrakeMode(mode);
}
