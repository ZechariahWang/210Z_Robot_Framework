#include "main.h"

// Motors and stuff
pros::Motor DriveFrontLeft(2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(18, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(15, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterIntake(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor InnerIntake(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntake(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::ADIDigitalOut Launcher('a');


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::ADIEncoder FrontAux('a', 'b', true);
pros::ADIEncoder ForwardAux('c', 'd', false);
pros::Imu imu_sensor(5);
pros::Imu imu_sensor_secondary(10);
pros::Vision vision_sensor(3);