#include "main.h"

// MAIN ROBOT GLOBALS
pros::Motor DriveFrontLeft(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(16, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidLeft(200, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidRight(201, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterShooter(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS); // REAL ONE
pros::Motor InnerShooter(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeBot(15, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeTop(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Pistons
pros::ADIDigitalOut Launcher('a');
pros::ADIDigitalOut Expansion('b');
pros::ADIDigitalOut LeftBrake('c');
pros::ADIDigitalOut RightBrake('d');
//
// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Sensors
pros::ADIEncoder FrontAux('e', 'f', true);
pros::ADIEncoder ForwardAux('g', 'h', false);
pros::Rotation RotationSensor(13);
pros::Imu imu_sensor(20);
pros::Imu imu_sensor_secondary(10);
pros::Vision vision_sensor(3);

// Switches
pros::ADIDigitalIn AutonSwitchForward('t');
pros::ADIDigitalIn AutonSwitchBackward('z');


// PROGRAMMING ROBOT GLOBALS
// pros::Motor DriveFrontLeft(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveFrontRight(20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackLeft(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackRight(4, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidLeft(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidRight(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

// pros::Motor OuterShooter(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor InnerShooter(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DiskIntakeBot(15, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DiskIntakeTop(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// // // Pistons
// pros::ADIDigitalOut Launcher('n');
// pros::ADIDigitalOut Expansion('x');
// pros::ADIDigitalOut LeftBrake('k');
// pros::ADIDigitalOut RightBrake('p');

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