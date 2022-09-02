#include "main.h"

unsigned short int powerSet = 100;
unsigned short int LauncherCounter = 0;

const short int maxPower = 100;
const short int halfPower = 75;
const short int lowPower = 50;
const unsigned int DriveTrainMultiplier = 94;


void SetDrive(int left, int right){
    DriveFrontLeft.move_voltage(left);
    DriveBackLeft.move_voltage(left);
    DriveFrontRight.move_voltage(right);
    DriveBackRight.move_voltage(right);
}

// The og code no cap
void Op_DTControl::HDriveControl(){
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

    double left = (rightXjoystick + leftYjoystick) * DriveTrainMultiplier;
    double right = (leftYjoystick - rightXjoystick) * DriveTrainMultiplier;

    SetDrive(left, right);
}

void Op_PowerShooter::PowerShooter(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        if (powerSet == maxPower){
            OuterIntake.move_voltage(12000);
            InnerIntake.move_voltage(12000);
        }
        else if (powerSet == halfPower){
            OuterIntake.move_voltage(12000 * halfPower);
            InnerIntake.move_voltage(12000 * halfPower);
        }
        else if (powerSet == lowPower){
            OuterIntake.move_voltage(12000 * lowPower);
            InnerIntake.move_voltage(12000 * lowPower); 
        }
    }
    else{
        OuterIntake.move_voltage(0);
        InnerIntake.move_voltage(0);
    }
}

void Op_PowerIntake::PowerIntake(){
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){
        DiskIntake.move_voltage(12000);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
         DiskIntake.move_voltage(-12000);
    }
    else{
        DiskIntake.move_voltage(0);
    }
}

void Op_LaunchDisk::LaunchDisk(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        Launcher.set_value(true);
        LauncherCounter++;
        if (LauncherCounter >= 5){
            Launcher.set_value(false);  
        }
    }
}

void Op_SetPowerAmount::SetPowerAmount(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        powerSet = maxPower;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        powerSet = halfPower;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        powerSet = lowPower;
    }
}

// these 2 functions are prob not gonna be implemented so no classes or inheritence are needed

void TurnToPointControl(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){ 
    }
}

void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        gx = 0;
        gy = 0;
    }
    else{
        std::cout << "not initiated yet" << std::endl;
    }
}



