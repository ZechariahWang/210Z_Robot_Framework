#include "main.h"

// MAIN ROBOT GLOBALS
pros::Motor DriveFrontLeft(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidLeft(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidRight(16, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterShooter(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS); // REAL ONE
pros::Motor InnerShooter(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntake(15, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Pistons
pros::ADIDigitalOut Launcher('a');

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Sensors
pros::ADIEncoder FrontAux('e', 'g', true);
pros::ADIEncoder ForwardAux('c', 'd', false);
pros::Rotation RotationSensor(13);
pros::Imu imu_sensor(20);
pros::Imu imu_sensor_secondary(10);
pros::Vision vision_sensor(3);

// Switches
pros::ADIDigitalIn AutonSwitchForward('h');
pros::ADIDigitalIn AutonSwitchBackward('b');


// PROGRAMMING ROBOT GLOBALS
// pros::Motor DriveFrontLeft(11, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveFrontRight(20, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackLeft(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackRight(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidLeft(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidRight(16, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// pros::Motor OuterShooter(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor InnerShooter(12, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DiskIntake(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// // Pistons
// pros::ADIDigitalOut Launcher('f');

// // Controller
// pros::Controller controller(pros::E_CONTROLLER_MASTER);

// // Sensors
// pros::ADIEncoder FrontAux('e', 'g', true);
// pros::ADIEncoder ForwardAux('c', 'd', false);
// pros::Rotation RotationSensor(19);
// pros::Imu imu_sensor(1);
// pros::Imu imu_sensor_secondary(10);
// pros::Vision vision_sensor(3);

// // Switches
// pros::ADIDigitalIn AutonSwitchForward('a');
// pros::ADIDigitalIn AutonSwitchBackward('b');