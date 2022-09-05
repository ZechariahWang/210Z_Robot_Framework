#include "main.h"

// Motors and stuff
pros::Motor DriveFrontLeft(10, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(20, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterIntake(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor InnerIntake(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntake(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::ADIDigitalOut Launcher('f');


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::ADIEncoder FrontAux('e', 'g', true);
pros::ADIEncoder ForwardAux('c', 'd', false);

pros::Imu imu_sensor(1);
pros::Imu imu_sensor_secondary(10);
pros::Vision vision_sensor(3);
pros::ADIDigitalIn AutonSwitchForward('a');
pros::ADIDigitalIn AutonSwitchBackward('b');