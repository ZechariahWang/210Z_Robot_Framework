#include "main.h"
#include "pros/motors.h"

// Values for altering
double powerSet = 0.6;
double LauncherCounter = 0;
double shootDelay = 200;

double shooterGain = 6;
double shooterOutput = 0;
double shooterPrevError = 0;
double tbh = 0;

// Constant voltage and velocity powers
const double maxPower = 100;
const double halfPower = 75;
const short int lowPower = 50;
const double DriveTrainMultiplier = 94;
const double FrontDriveTrainMultiplier = 94;
const double BackDriveTrainMultiplier = 94;

// Move motors the given power amounts
void SetDrive(int left, int right){
    DriveFrontLeft.move_voltage(left);
    DriveBackLeft.move_voltage(left);
    DriveMidLeft.move_voltage(left);
    DriveFrontRight.move_voltage(right);
    DriveBackRight.move_voltage(right);
    DriveMidRight.move_voltage(right);
}

// The og code, standard h-drive control
void Op_DTControl::HDriveControl(){
    double leftYjoystick  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    double leftXjoystick  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    double rightYjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    double rightXjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    if(fabs(leftYjoystick) < 10){
        leftYjoystick = 0;
    }
   if(fabs(rightYjoystick) < 10){
        rightYjoystick = 0;
    }

    double left = (rightXjoystick + leftYjoystick) * DriveTrainMultiplier;
    double right = (leftYjoystick - rightXjoystick) * DriveTrainMultiplier;
    SetDrive(left, right);
}

void Op_PowerShooter::TBH_AlgorithmControl(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        int maxSpeed = 600 * powerSet;
        double currentSpeed = (OuterShooter.get_actual_velocity());
        char buffer[300];
        sprintf(buffer, SYMBOL_UP " current speed %f:", currentSpeed); // Confirm all debug data system fully operational
        lv_label_set_text(debugLine1, buffer);
        double error = maxSpeed - currentSpeed;
        shooterOutput += shooterGain * error;
        if (utility::sgn(error) != utility::sgn(shooterPrevError)){
            shooterOutput = 0.5 * (shooterOutput + tbh);
            tbh = shooterOutput;
            shooterPrevError = error;
        }
        OuterShooter.move_velocity(shooterOutput);
        InnerShooter.move_velocity(shooterOutput);
    }
    else{
        OuterShooter.move_velocity(0);
        InnerShooter.move_velocity(0);
    }
}

// Power shooter function
void Op_PowerShooter::PowerShooter(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        OuterShooter.move_voltage(12000 * powerSet);
        InnerShooter.move_voltage(12000 * powerSet);
    }
    else{
        OuterShooter.move_voltage(0);
        InnerShooter.move_voltage(0);
    }
}

// Power intake function
void Op_PowerIntake::PowerIntake(){
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))){
        DiskIntakeTop.move_voltage(12000);
        DiskIntakeBot.move_voltage(12000);

    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        DiskIntakeTop.move_voltage(-12000);
        DiskIntakeBot.move_voltage(-12000);
    }
    else{
        DiskIntakeTop.move_voltage(0);
        DiskIntakeBot.move_voltage(0);
    }
}

// Launch disk/piston control function
static bool launchStatus = false;
unsigned short int counterDisk = 0;
unsigned short int sequenceDelay = 0;
void Op_LaunchDisk::LaunchDisk(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        launchStatus = !launchStatus;
        Launcher.set_value(!launchStatus);
        pros::delay(shootDelay);
        launchStatus = false;
        Launcher.set_value(!launchStatus); 
        pros::delay(shootDelay);
    }
}

// Function for changing power of flywheel
double currentPower = 100;
bool maxPowerEnabled = true;
void Op_SetPowerAmount::SetPowerAmount(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        maxPowerEnabled = !maxPowerEnabled;
        if (maxPowerEnabled){
            powerSet = 0.6;
        }
        else {
            powerSet = 0.8;
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        powerSet += 0.05;
        if (powerSet > 1){
            powerSet = 0;
        }
        else if (powerSet < 0){
            powerSet = 1;
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        powerSet -= 0.05;
        if (powerSet > 1){
            powerSet = 0;
        }
        else if (powerSet < 0){
            powerSet = 1;
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        shootDelay -= 50;
        if (shootDelay < 100)
            shootDelay = 200;
        if (shootDelay > 200) 
            shootDelay = 200;
    }
    controller.print(1, 0, "FW: %.2f", powerSet);
    controller.print(1, 1, "SD: %f", shootDelay);
}

static bool robotBrakeType = false;
void Op_SetMotorType::setMotorType(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        robotBrakeType = !robotBrakeType;
    }
    if (robotBrakeType){
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftBrake.set_value(true);
        RightBrake.set_value(true);
    }
    else {
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftBrake.set_value(false);
        RightBrake.set_value(false);
    }
}

bool expansionSet = false;
void Op_EndGame::InitiateExpansion(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        Expansion.set_value(expansionSet);
        expansionSet = !expansionSet;
    }
}
void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        gx = 0;
        gy = 0;
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

    if (reversed){ // kevin is homosexual

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
}



