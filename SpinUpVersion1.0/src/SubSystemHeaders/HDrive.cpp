// Import Libraries \\

#include "main.h"
#include "math.h"
#include "cmath"
#include "iostream"

#define public

const unsigned int DriveTrainMultiplier = 94;

void SetDrive(int left, int right){

    DriveFrontLeft.move_voltage(left);
    DriveBackLeft.move_voltage(left);
    DriveFrontRight.move_voltage(right);
    DriveBackRight.move_voltage(right);

}


// The og code no cap
void HDriveControl(){
    double leftYjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    double leftXjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    double rightYjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    double rightXjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1

    if(abs(leftYjoystick) < 10){
        leftYjoystick = 0;
    }

   if(abs(rightYjoystick) < 10){
        rightYjoystick = 0;
    }

    double left = (leftXjoystick + leftYjoystick) * DriveTrainMultiplier;
    double right = (leftYjoystick - leftXjoystick) * DriveTrainMultiplier;

    SetDrive(left, right);
}