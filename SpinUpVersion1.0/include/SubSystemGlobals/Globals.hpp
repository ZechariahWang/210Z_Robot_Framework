// Motors lolllll

#include "main.h"

extern pros::Motor DriveFrontLeft;
extern pros::Motor DriveFrontRight;
extern pros::Motor DriveBackLeft;
extern pros::Motor DriveBackRight;
extern pros::Motor DriveMidLeft;
extern pros::Motor DriveMidRight;
extern pros::ADIEncoder FrontAux;
extern pros::ADIEncoder ForwardAux;
extern pros::Imu imu_sensor;
extern pros::Imu imu_sensor_secondary;
extern pros::Vision vision_sensor;
extern pros::Rotation RotationSensor;

extern pros::Motor OuterShooter;
extern pros::Motor InnerShooter;
extern pros::Motor DiskIntakeBot;
extern pros::Motor DiskIntakeTop;

extern pros::ADIDigitalOut Launcher;
extern pros::ADIDigitalOut Expansion;
extern pros::ADIDigitalOut LeftBrake;
extern pros::ADIDigitalOut RightBrake;

extern pros::ADIDigitalIn AutonSwitchForward;
extern pros::ADIDigitalIn AutonSwitchBackward;
extern pros::Controller controller;

class Global {
    private:
        int init;

    public:
        double ImuMonitor();

};
