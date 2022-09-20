#include "main.h"

unsigned short int SelectedAuton = 0; // Auton choice
unsigned short int AutonFinalized = 0; // 0 = false, 1 = true

const unsigned short int MaxLimit = 11; // The max limit switches can go up to
const unsigned short int MinLimit = 0; // The min limit switches can go up to

unsigned short int globalAuton = 1; // Different auton function depending on selected auton
unsigned short int counterForward1 = 0;
unsigned short int counterBackward2 = 0;
unsigned short int simultaneousInputLimit = 20;

static bool pressed1 = true;
static bool pressed2 = true;

static bool currentlyPressed1 = false;
static bool currentlyPressed2 = false;


// This funcion receieves input from switches on robot. Used to determine which auton to use. Press middle button to finalize choice
void Init_AutonSwitchMain::ReceiveInput(long int time){
    FinalizeAuton data;
    int currentTime = 0;

    while (currentTime <= time){
    	data.DisplayData();

        if (currentlyPressed1){
            counterForward1 += 1;
            pressed1 = false;
            if (counterForward1 > 20){
                pressed1 = true;
                counterForward1 = 0;
                currentlyPressed1 = false;
            }
        }
        else if (currentlyPressed2){
            counterBackward2 += 1;
            pressed2 = false;
            if (counterBackward2 > 20){
                pressed2 = true;
                counterBackward2 = 0;
                currentlyPressed2 = false;
            }
        }

        if (AutonSwitchForward.get_new_press()){
            SelectedAuton += 1;
            currentlyPressed1 = true;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }
        else if (AutonSwitchBackward.get_new_press()){
            SelectedAuton -= 1;
            currentlyPressed2 = true;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 0;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }

        if (pressed2 == false && pressed1 == false){
            pros::lcd::print(7, "Finalized Auton Choice: %d", SelectedAuton);
		    pros::delay(2000);
		    pros::lcd::print(7, "Waiting for game phase...");
            break;
        }

        if (AutonFinalized == 1){
        	pros::lcd::print(7, "Finalized Auton Choice: %d", SelectedAuton);
		    pros::delay(2000);
		    pros::lcd::print(7, "Waiting for game phase...");
            break;
        }

        currentTime += 10;
        pros::delay(10);
    }
}

// This function is for the auton selector, however with no time limit on choosing the desired auton. Will only break out once middle button is pressed.
void Init_AutonSwitchMain::ReceiveInput_noLimit(long int time){
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
        else if (AutonSwitchBackward.get_new_press() && AutonSwitchForward.get_new_press()){
            break;
        }

        if (AutonFinalized == 1){
        	pros::lcd::print(7, "Finalized Auton Choice: %d", globalAuton);
		    pros::delay(2000);
		    pros::lcd::print(7, "Waiting for game phase...");
            break;
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

void FinalizeAuton::DisplayCurrentAuton(){
    pros::lcd::print(7, "Final Chosen Auton: %d", SelectedAuton);
}

void FinalizeAuton::DisplayData(){
    pros::lcd::print(1, "X:%.1f, Y:%.1f, T:%.1f", ceil(gx), ceil(gy), ceil(ImuMon())); // Odometry Values
    pros::lcd::print(2, "FL Temp:%.1f, BL Temp:%.1f", DriveFrontLeft.get_temperature(), DriveBackLeft.get_temperature()); // Left motor status
    pros::lcd::print(3, "FR Temp:%.1f, BR Temp:%.1f", DriveFrontRight.get_temperature(), DriveBackRight.get_temperature()); // Right motor status
    pros::lcd::print(4, "Official Selected Auton Type: %d", SelectedAuton); // The offical auton that will be called during match
}