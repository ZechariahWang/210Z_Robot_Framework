#include "main.h"

unsigned short int SelectedAuton = 0; // Auton choice

const unsigned short int MaxLimit = 10; // The max limit switches can go up to
const unsigned short int MinLimit = 0; // The min limit switches can go up to

short int globalAuton = 1; // Different auton function depending on selected auton

// Receieves input from switches on robot. Used to determine which auton to use.
void Init_AutonSwitchMain::ReceiveInput(long int time){
    int currentTime = 0;

    while (currentTime <= time){
        if (AutonSwitchForward.get_new_press()){
            SelectedAuton++;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }
        else if (AutonSwitchBackward.get_new_press()){
            SelectedAuton--;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }

        pros::lcd::print(4, "Current Auton: %d", SelectedAuton);
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
    case 0:
        pros::lcd::print(5, "Auton 0 selected");
        globalAuton = 0;
        AutonSelectorPrimary(0);
        break;
    case 1:
        pros::lcd::print(5, "Auton 1 selected");
        globalAuton = 1;
        AutonSelectorPrimary(1);
        break;
    case 2:
        pros::lcd::print(5, "Auton 2 selected");
        AutonSelectorPrimary(2);
        globalAuton = 2;
        break;
    case 3:
        pros::lcd::print(5, "Auton 3 selected");
        AutonSelectorPrimary(3);
        globalAuton = 3;
        break;
    case 4:
        pros::lcd::print(5, "Auton 4 selected");
        AutonSelectorPrimary(4);
        globalAuton = 4;
        break;
    case 5:
        pros::lcd::print(5, "Auton 5 selected");
        AutonSelectorPrimary(5);
        globalAuton = 5;
        break;
    case 6:
        pros::lcd::print(5, "Auton 6 selected");
        AutonSelectorPrimary(6);
        globalAuton = 6;
        break;
    case 7:
        pros::lcd::print(5, "Auton 7 selected");
        AutonSelectorPrimary(7);
        globalAuton = 7;
        break;
    case 8:
        pros::lcd::print(5, "Auton 8 selected");
        AutonSelectorPrimary(8);
        globalAuton = 8;
        break;
    case 9:
        pros::lcd::print(5, "Auton 9 selected");
        AutonSelectorPrimary(9);
        globalAuton = 9;
        break;
    case 10:
        pros::lcd::print(5, "Auton 10 selected");
        AutonSelectorPrimary(10);
        globalAuton = 10;
        break;
    default:
         pros::lcd::print(5, "Default Auton selected");
         AutonSelectorPrimary(0);
         globalAuton = 0;
        break;
    }
}