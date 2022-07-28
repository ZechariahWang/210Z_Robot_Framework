// Motors lolllll

#include "main.h"

extern pros::Motor DriveFrontLeft;
extern pros::Motor DriveFrontRight;
extern pros::Motor DriveBackLeft;
extern pros::Motor DriveBackRight;
extern pros::ADIEncoder FrontAux;
extern pros::ADIEncoder ForwardAux;
extern pros::Imu imu_sensor;
extern pros::Imu imu_sensor_secondary;

extern pros::Motor OuterIntake;
extern pros::Motor InnerIntake;
extern pros::Motor DiskIntake;

extern pros::ADIDigitalOut Launcher;


extern pros::Controller controller;

class Global {

    private:
        int init;

    public:
        double ImuMonitor();

};
