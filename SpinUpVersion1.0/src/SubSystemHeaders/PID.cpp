#include "main.h"
#include "vector"
#include "variant"
#include "array"


////////////////////////////////////////////////*/
/* Section: Global Utility Functions
///////////////////////////////////////////////*/

namespace utility
{
  int sgn(double num){
    return (num < 0) ? -1 : ((num > 0) ? 1 : 0); // Returns -1 if num is negative, and 1 if num is HIV positive.
  }
  void stop(){
    DriveFrontLeft.move_voltage(0);
    DriveBackLeft.move_voltage(0);
    DriveFrontRight.move_voltage(0);
    DriveBackRight.move_voltage(0);
  }

  void leftvreq(double voltage){
    DriveFrontLeft.move_voltage(voltage);
    DriveBackLeft.move_voltage(voltage);
  }

  void rightvreq(double voltage){
    DriveFrontRight.move_voltage(voltage);
    DriveBackRight.move_voltage(voltage);
  }

  void fullreset(double resetval, bool imu){
    DriveFrontLeft.set_zero_position(resetval);
    DriveBackLeft.set_zero_position(resetval);
    DriveFrontRight.set_zero_position(resetval);
    DriveBackRight.set_zero_position(resetval);

    if (imu == true){
      imu_sensor.tare_rotation();
    }
  }
}

////////////////////////////////////////////////*/
/* Section: Translation PID
///////////////////////////////////////////////*/

// PID Settings

const double kp = 0.150; // 0.4
const double ki = 0.00001;
const double kd = 0.82;

double derivative          = 0;
double integral            = 0;
double tolerance           = 50;
double error               = 0;
double previouserror       = 0;
double multiplier          = 200;
double maxSpeed            = 12000;
double averageposition     = 0;
double currentposition     = 0;
double averageHeading      = 0;
double FailSafeCounter     = 0;
int threshholdcounter      = 0;

// Kind of has correction now? very scuffed, might need to turn kp down later
void ForwardPID(int target){

  pros::lcd::print(5, " ");
  utility::fullreset(0, true);
  error = 0;
  previouserror = 0;
  integral = 0;
  derivative = 0;
  FailSafeCounter = 0;

  averageposition = (DriveFrontRight.get_position() + DriveFrontLeft.get_position()) / 2; // Getting average position of drivetrain
  
  while(true){

    SecondOdometry();
    pros::lcd::print(1, "raw pos: %f ", averageposition); // Debugging 
    averageHeading = imu_sensor.get_rotation(); // Getting average heading of imu
    currentposition = (DriveFrontRight.get_position() + DriveFrontLeft.get_position()) / 2; // Getting average position of drivetrain
    error = target - (currentposition - averageposition); // Getting error between distance of target and robot
    integral += error; // Adding area (integral) between each iteration
    pros::lcd::print(2, "error: %f ", error); // Debugging

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (error == 0 || error > target) {
      integral = 0;
    }

    derivative = error - previouserror; // Calculating the rate of change in error 
    pros::lcd::print(4, "error - prev: %f ", error - previouserror); // Debugging
    previouserror = error;

    double voltage = ((error * kp) + (integral * ki) + (derivative * kd)) * 94; // Merging all calculations into final voltage power
    pros::lcd::print(3, "voltage: %f ", voltage); // Debugging

    if (voltage > maxSpeed){
      voltage = maxSpeed;
    }

    double difference = DriveFrontLeft.get_position() - DriveFrontRight.get_position();
    double compensation = utility::sgn(difference);

    utility::rightvreq(voltage + compensation); // Making motors move amount in volts
    utility::leftvreq(voltage); // Making motors move amount in volts

    if(fabs(error) < tolerance){
      threshholdcounter++;
    }
    else{
      threshholdcounter = 0;
    }
    if (threshholdcounter > 10){
      utility::stop();
      break;
    }

    if (fabs(error - previouserror) < 0.3) {
      FailSafeCounter++;
    }
    else {
      FailSafeCounter = 0;
    }

    if (FailSafeCounter >= 300) {
      utility::stop();
      break;
    }
    pros::delay(10);

  }
  pros::lcd::print(6, "Drive PID sequence finished, exiting control.");
}

double ImuMonitorTheta() {
  double theta = imu_sensor.get_rotation();
 
  while (theta < 0) {
    theta += 360;
  }
  while (theta > 360) {
    theta -= 360;
  }
 
  return theta;
 
}

////////////////////////////////////////////////*/
/* Section: Rotation PID
///////////////////////////////////////////////*/

const double t_kp = 1.1;
const double t_ki = 0.001;
const double t_kd = 1.9;

double t_derivative          = 0;
double t_integral            = 0;
double t_tolerance           = 3;
double t_error               = 0;
double t_previouserror       = 0;
double t_multiplier          = 3000;
double t_averageposition     = 0;
double t_averageHeading      = 0;
double t_FailSafeCounter     = 0;
int t_threshholdcounter      = 0;

// pls why take so long to turn

float Turn_PID(double t_theta){

utility::fullreset(0, false);
  t_error = 0;
  t_previouserror = 0;
  t_integral = 0;
  t_derivative = 0;
  t_FailSafeCounter = 0;

    SecondOdometry();
    t_averageHeading = imu_sensor.get_rotation(); // Getting average heading of imu
    t_error = t_theta - t_averageHeading; // Getting error between distance of target and robot
    t_integral += t_error; // Adding area (integral) between each iteration

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (t_error == 0 || t_error > t_theta) {
      t_integral = 0;
    }

    t_derivative = t_error - t_previouserror; // Calculating the rate of change in error 
    t_previouserror = t_error;

    double voltage = ((t_error * t_kp) + (t_integral * t_ki) + (t_derivative * t_kd)) * 94; // Merging all calculations into final voltage power
    //pros::lcd::print(3, "voltage: %f ", voltage); // Debugging
    if(fabs(t_error) < t_tolerance){
      t_threshholdcounter++;
    }
    else{
      t_threshholdcounter = 0;
    }

    if (fabs(t_error - t_previouserror) < 0.3) {
      t_FailSafeCounter++;
    }
    else {
      t_FailSafeCounter = 0;
    }

  return voltage;

}
void TurnPID(double t_theta){

  utility::fullreset(0, false);
  t_error = 0;
  t_previouserror = 0;
  t_integral = 0;
  t_derivative = 0;
  t_FailSafeCounter = 0;

  while(true){ 

    SecondOdometry();
    t_averageHeading = imu_sensor.get_rotation(); // Getting average heading of imu
    pros::lcd::print(4, "Heading: %f ", t_averageHeading); // Debugging
    t_error = t_theta - t_averageHeading; // Getting error between distance of target and robot
    t_integral += t_error; // Adding area (integral) between each iteration
    pros::lcd::print(5, "turn error: %f ", t_error); // Debugging
    pros::lcd::print(7, "Target theta: %f ", t_theta); // Debugging

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (t_error == 0 || t_error > t_theta) {
      t_integral = 0;
    }

    t_derivative = t_error - t_previouserror; // Calculating the rate of change in error 
    t_previouserror = t_error;

    double voltage = ((t_error * t_kp) + (t_integral * t_ki) + (t_derivative * t_kd)) * 94; // Merging all calculations into final voltage power
    //pros::lcd::print(3, "voltage: %f ", voltage); // Debugging

    utility::leftvreq(voltage); // Making motors move amount in volts
    utility::rightvreq(voltage * -1); // Making motors move amount in volts

    if(fabs(t_error) < t_tolerance){
      t_threshholdcounter++;
    }
    else{
      t_threshholdcounter = 0;
    }
    if (t_threshholdcounter > 10){
      utility::stop();
      pros::lcd::print(6, "Broke out: %f "); // Debugging
      break;
    }

    if (fabs(t_error - t_previouserror) < 0.3) {
      t_FailSafeCounter++;
    }
    else {
      t_FailSafeCounter = 0;
    }

    if (t_FailSafeCounter >= 300) {
      utility::stop();
      break;
    }
    pros::delay(10);

  }
  pros::lcd::print(6, "Turn PID sequence finished, exiting control.");

}