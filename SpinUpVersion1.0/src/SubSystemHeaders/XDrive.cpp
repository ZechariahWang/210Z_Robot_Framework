// Import Libraries \\

#include "main.h"
#include "math.h"
#include "cmath"
#include "iostream"

#define public


const unsigned int DriveTrainMultiplier = 94;


// What the hell is going on here
void XDriveTrainControl() {

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
