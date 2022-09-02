// Import Libraries \\

#include "main.h"
#include "math.h"
#include "cmath"
#include "iostream"

#define public

const unsigned int FrontDriveTrainMultiplier = 95;
const unsigned int BackDriveTrainMultiplier = 95;

static bool reversed = false;


// cancer
void MecanumDriveControl(){

    double vertical;
    double horizontal;
    double pivot;
    
    vertical = (double)(-controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // CH3
    horizontal = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // CH4
    pivot = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // CH1

    double leftfrontpower = pivot + (vertical + horizontal);
    double rightbackpower = pivot + (vertical - horizontal);
    double leftbackpower = pivot - (vertical - horizontal);
    double rightfrontpower = pivot - (vertical + horizontal);

    if (reversed){

        vertical = (double)(-controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // CH3
        horizontal = (double)(-controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // CH4
        pivot = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // CH1

        double leftfrontpower = pivot + (vertical + horizontal);
        double rightbackpower = pivot + (vertical - horizontal);
        double leftbackpower = pivot - (vertical - horizontal);
        double rightfrontpower = pivot - (vertical + horizontal);

        DriveFrontLeft.move_voltage(-leftfrontpower * FrontDriveTrainMultiplier);
        DriveBackRight.move_voltage(-rightbackpower * BackDriveTrainMultiplier);
        DriveBackLeft.move_voltage(-leftbackpower * BackDriveTrainMultiplier);
        DriveFrontRight.move_voltage(-rightfrontpower * FrontDriveTrainMultiplier);
    }
    else{

        vertical = (double)(-controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // CH3
        horizontal = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // CH4
        pivot = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // CH1

        double leftfrontpower = pivot + (vertical + horizontal);
        double rightbackpower = pivot + (vertical - horizontal);
        double leftbackpower = pivot - (vertical - horizontal);
        double rightfrontpower = pivot - (vertical + horizontal);

        DriveFrontLeft.move_voltage(leftfrontpower * FrontDriveTrainMultiplier);
        DriveBackRight.move_voltage(rightbackpower * BackDriveTrainMultiplier);
        DriveBackLeft.move_voltage(leftbackpower * BackDriveTrainMultiplier);
        DriveFrontRight.move_voltage(rightfrontpower * FrontDriveTrainMultiplier);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
	    reversed = !reversed;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
    }
}