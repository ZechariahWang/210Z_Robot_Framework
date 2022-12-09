#include "main.h"

unsigned short int SelectedAuton = 1; // Auton choice
unsigned short int AutonFinalized = 0; // 0 = false, 1 = true

const unsigned short int MaxLimit = 11; // The max limit switches can go up to
const unsigned short int MinLimit = 0; // The min limit switches can go up to

unsigned short int globalAuton = 1; // Different auton function depending on selected auton
unsigned short int counterForward1 = 0; // Forward counter
unsigned short int counterBackward2 = 0; // Reverse Counter
unsigned short int simultaneousInputLimit = 20; // Limit before simultaneous switch is reverted back

static bool pressed1 = true; // Status of forward switch
static bool pressed2 = true; // Status of backward switch

static bool currentlyPressed1 = false; // Local switch status
static bool currentlyPressed2 = false; // Local switch status


// This funcion receieves input from switches on robot. Used to determine which auton to use. Press both buttons at the same time OR LCD middle button to finalize choice.
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
            currentlyPressed1 = true;
            SelectedAuton += 1;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 1;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }
        else if (AutonSwitchBackward.get_new_press()){
            currentlyPressed2 = true;
            SelectedAuton -= 1;
            if (SelectedAuton >= MaxLimit){
                SelectedAuton = 1;
            }
            else if (SelectedAuton <= MinLimit){
                SelectedAuton = 10;
            }
        }

        if (pressed2 == false && pressed1 == false){
            SelectedAuton += 1;
            char buffer[100];
        	sprintf(buffer, "Chosen Auton: %d", SelectedAuton);
	        lv_label_set_text(displayDataL1, buffer);
		    pros::delay(2000);
        	sprintf(buffer, "Entering game phase...");
	        lv_label_set_text(displayDataL1, buffer);
            break;
            // f
        }

        if (AutonFinalized == 1){
            char buffer[100];
        	sprintf(buffer, "Chosen Auton: %d", SelectedAuton);
	        lv_label_set_text(displayDataL1, buffer);
		    pros::delay(2000);
        	sprintf(buffer, "Entering game phase...");
	        lv_label_set_text(displayDataL1, buffer);
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
		    pros::delay(2000);
		   // pros::lcd::print(7, "Entering game phase...");
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
    Launcher.set_value(true);
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

// Display chosen auton
void FinalizeAuton::DisplayCurrentAuton(){
    //pros::lcd::print(7, "Final Chosen Auton: %d", SelectedAuton);
}

// Display metrics/robot data
void FinalizeAuton::DisplayData(){
	char buffer[300];

	sprintf(buffer, SYMBOL_GPS " X: %.2f Y: %.2f Theta: %f", gx, gy, ImuMon());
	lv_label_set_text(displayDataL5, buffer);

	sprintf(buffer, SYMBOL_WARNING " FL: %.2f BL: %.2f", DriveFrontLeft.get_temperature(), DriveBackLeft.get_temperature());
	lv_label_set_text(displayDataL4, buffer);

	sprintf(buffer, SYMBOL_WARNING " FR: %.2f BR: %.2f", DriveFrontRight.get_temperature(), DriveBackRight.get_temperature());
	lv_label_set_text(displayDataL3, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Current Selected Auton Type: %d", SelectedAuton);
	lv_label_set_text(displayDataL2, buffer);
}