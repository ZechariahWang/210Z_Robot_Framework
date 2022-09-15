#include "main.h"

unsigned short int SelectedAuton = 0; // Auton choice

const unsigned short int MaxLimit = 11; // The max limit switches can go up to
const unsigned short int MinLimit = 0; // The min limit switches can go up to

short int globalAuton = 1; // Different auton function depending on selected auton

// Receieves input from switches on robot. Used to determine which auton to use.

void breakOut() {
    pros::lcd::print(5, "kfjsdn:");
}

void Init_AutonSwitchMain::ReceiveInput(long int time){
    FinalizeAuton data;
    int currentTime = 0;

    while (currentTime <= time){
    	data.DisplayData();
        if (AutonSwitchForward.get_new_press()){
            SelectedAuton += 1;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }
        else if (AutonSwitchBackward.get_new_press()){
            SelectedAuton -= 1;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }

        currentTime += 10;
        pros::delay(10);
    }
}

// Reset all sensors used in autonomous routines
void ResetSensors::ResetAllPrimarySensors(){
    imu_sensor.tare_rotation();
    RotationSensor.reset_position();
    DriveFrontLeft.set_zero_position(0);
    DriveFrontRight.set_zero_position(0);
    DriveBackLeft.set_zero_position(0);
    DriveBackRight.set_zero_position(0);
    gx = 0;
    gy = 0;
}


// Finalize auton choices
void FinalizeAuton::SelectAuton(){

    int chosenAuton = SelectedAuton;
    switch (chosenAuton)
    {
    case 0: // Skills
        globalAuton = 0;
        AutonSelectorPrimary(0);
        break;
    case 1:
        globalAuton = 1;
        AutonSelectorPrimary(1);
        break;
    case 2:
        AutonSelectorPrimary(2);
        globalAuton = 2;
        break;
    case 3:
        AutonSelectorPrimary(3);
        globalAuton = 3;
        break;
    case 4:
        AutonSelectorPrimary(4);
        globalAuton = 4;
        break;
    case 5:
        AutonSelectorPrimary(5);
        globalAuton = 5;
        break;
    case 6:
        AutonSelectorPrimary(6);
        globalAuton = 6;
        break;
    case 7:
        AutonSelectorPrimary(7);
        globalAuton = 7;
        break;
    case 8:
        AutonSelectorPrimary(8);
        globalAuton = 8;
        break;
    case 9:
        AutonSelectorPrimary(9);
        globalAuton = 9;
        break;
    case 10:
        AutonSelectorPrimary(10);
        globalAuton = 10;
        break;
    default:
         AutonSelectorPrimary(0);
         globalAuton = 0;
        break;
    }
}

void FinalizeAuton::DisplayData(){
    pros::lcd::print(1, "X:%.1f, Y:%.1f, T:%.1f", ceil(gx), ceil(gy), ceil(ImuMon())); // Odometry Values
    pros::lcd::print(2, "FL Temp:%.1f, BL Temp:%.1f", DriveFrontLeft.get_temperature(), DriveBackLeft.get_temperature()); // Left motor status
    pros::lcd::print(3, "FR Temp:%.1f, BR Temp:%.1f", DriveFrontRight.get_temperature(), DriveBackRight.get_temperature()); // Right motor status
    pros::lcd::print(4, "Selected Auton: %d", SelectedAuton);
}