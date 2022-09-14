#include "main.h"

// Values for altering
unsigned short int powerSet = 100;
unsigned short int LauncherCounter = 0;

// Constant voltage and velocity powers
const short int maxPower = 100;
const short int halfPower = 75;
const short int lowPower = 50;
const unsigned int DriveTrainMultiplier = 94;
const unsigned int FrontDriveTrainMultiplier = 94;
const unsigned int BackDriveTrainMultiplier = 94;

// Move motors the given power amounts
void SetDrive(int left, int right){
    DriveFrontLeft.move_voltage(left);
    DriveBackLeft.move_voltage(left);
    DriveFrontRight.move_voltage(right);
    DriveBackRight.move_voltage(right);
}

// The og code, standard h-drive control
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

// Power shooter function
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

// Power intake function
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

// Launch disk/piston control function
void Op_LaunchDisk::LaunchDisk(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        Launcher.set_value(true);
        LauncherCounter++;
        if (LauncherCounter >= 5){
            Launcher.set_value(false);  
        }
    }
}

// Function for changing power of flywheel
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

// these 2 functions are prob not gonna be implemented on H-Drive so no classes or inheritence are needed
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

// X Drive code with limits on voltage powers
void Op_DTControl::XDriveTrainControl() {

  double front_left  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double back_left   = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double front_right = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double back_right  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        
  double max_raw_sum = (double)(abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
  double max_XYstick_value = (double)(std::max(abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))));
        
  if (max_raw_sum != 0) {
    front_left  = front_left / max_raw_sum * max_XYstick_value;
    back_left   = back_left / max_raw_sum * max_XYstick_value;
    front_right = front_right / max_raw_sum * max_XYstick_value;
    back_right  = back_right / max_raw_sum * max_XYstick_value;
  }
        
  front_left  = front_left  + controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  back_left   = back_left   + controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  front_right = front_right - controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  back_right  = back_right  - controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
  max_raw_sum = std::max(std::abs(front_left), std::max(std::abs(back_left), std::max(std::abs(front_right), std::max(std::abs(back_right), 100.0))));
        
  front_left  = front_left  / max_raw_sum * 100.0;
  back_left   = back_left   / max_raw_sum * 100.0;
  front_right = front_right / max_raw_sum * 100.0;
  back_right  = back_right  / max_raw_sum * 100.0;

  DriveFrontLeft.move_voltage(front_left * DriveTrainMultiplier);
  DriveBackLeft.move_voltage(back_left * DriveTrainMultiplier);
  DriveFrontRight.move_voltage(front_right * DriveTrainMultiplier);
  DriveBackRight.move_voltage(back_right * DriveTrainMultiplier);

}

// Mecanum Drive control with no voltage limits, and a reversed option thanks to kevin
bool reversed = false;
void Op_DTControl::MecanumDriveControl(){

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



