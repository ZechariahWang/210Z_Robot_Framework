#include "main.h"

unsigned short int powerSet = 100;
unsigned short int LauncherCounter = 0;

const short int maxPower = 100;
const short int halfPower = 75;
const short int lowPower = 50;

void PowerShooter(){
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

void PowerIntake(){
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

void LaunchDisk(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        Launcher.set_value(true);
        LauncherCounter++;
        if (LauncherCounter >= 5){
            Launcher.set_value(false);  
        }
    }
}

void SetPowerAmount(){
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

void TurnToPointControl(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
	    TurnToPoint(50, 2);
    }
}

void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
	    // TurnPID(0);
        gx = 0;
        gy = 0;
    }
    else{
        std::cout << "not initiated yet" << std::endl;
    }
}



