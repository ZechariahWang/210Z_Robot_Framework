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
void ForwardPID(int target, int maxVoltage){

  pros::lcd::print(5, " ");
  utility::fullreset(0, false);
  error = 0;
  previouserror = 0;
  integral = 0;
  derivative = 0;
  FailSafeCounter = 0;

  averageposition = (DriveFrontRight.get_position() + DriveFrontLeft.get_position()) / 2; // Getting average position of drivetrain
  
  while(true){

    SecondOdometry();
    //pros::lcd::print(1, "raw pos: %f ", averageposition); // Debugging 
    averageHeading = imu_sensor.get_rotation(); // Getting average heading of imu
    currentposition = (DriveFrontRight.get_position() + DriveFrontLeft.get_position()) / 2; // Getting average position of drivetrain
    error = target - (currentposition - averageposition); // Getting error between distance of target and robot
    integral += error; // Adding area (integral) between each iteration
    //pros::lcd::print(2, "error: %f ", error); // Debugging

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (error == 0 || error > target) {
      integral = 0;
    }

    derivative = error - previouserror; // Calculating the rate of change in error 
    //pros::lcd::print(4, "error - prev: %f ", error - previouserror); // Debugging
    previouserror = error;

    double voltage = ((error * kp) + (integral * ki) + (derivative * kd)) * 94; // Merging all calculations into final voltage power
    //pros::lcd::print(3, "voltage: %f ", voltage); // Debugging

    if (voltage > maxVoltage){
      voltage = maxVoltage;
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
/* Section: Rotation PID GTC ONLY!
///////////////////////////////////////////////*/

const double GTC_kp = 2;
const double GTC_ki = 0.001;
const double GTC_kd = 1.9;

double GTC_derivative          = 0;
double GTC_integral            = 0;
double GTC_tolerance           = 3;
double GTC_error               = 0;
double GTC_previouserror       = 0;
double GTC_multiplier          = 3000;
double GTC_averageposition     = 0;
double GTC_averageHeading      = 0;
double GTC_FailSafeCounter     = 0;
int GTC_threshholdcounter      = 0;

// pls why take so long to turn
// Only for the GTC function in motionAlg. RETURNS a voltage value only
float Turn_PID(double GTC_theta){

  utility::fullreset(0, false);
  GTC_error = 0;
  GTC_previouserror = 0;
  GTC_integral = 0;
  GTC_derivative = 0;
  GTC_FailSafeCounter = 0;

    SecondOdometry();
    GTC_averageHeading = imu_sensor.get_rotation(); // Getting average heading of imu
    GTC_error = GTC_theta - GTC_averageHeading; // Getting error between distance of target and robot
    GTC_integral += GTC_error; // Adding area (integral) between each iteration

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (GTC_error == 0 || GTC_error > GTC_theta) {
      GTC_integral = 0;
    }

    GTC_derivative = GTC_error - GTC_previouserror; // Calculating the rate of change in error 
    GTC_previouserror = GTC_error;

    double voltage = (GTC_error * GTC_kp * 0.01) * 94; // Merging all calculations into final voltage power
    //pros::lcd::print(3, "voltage: %f ", voltage); // Debugging
    if(fabs(GTC_error) < GTC_tolerance){
      GTC_threshholdcounter++;
    }
    else{
      GTC_threshholdcounter = 0;
    }

    if (fabs(GTC_error - GTC_previouserror) < 0.3) {
      GTC_FailSafeCounter++;
    }
    else {
      GTC_FailSafeCounter = 0;
    }

  return voltage;

}

////////////////////////////////////////////////*/
/* Section: Rotation PID
///////////////////////////////////////////////*/

const double t_kp = 2;
const double t_ki = 0.001;
const double t_kd = 1.9;

double t_derivative          = 0;
double t_integral            = 0;
double t_tolerance           = 12;
double t_error               = 0;
double t_previouserror       = 0;
double t_multiplier          = 3000;
double t_averageposition     = 0;
double t_averageHeading      = 0;
double t_FailSafeCounter     = 0;
int t_threshholdcounter      = 0;


// When turning only use this one not the above one
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
    //pros::lcd::print(4, "Heading: %f ", t_averageHeading); // Debugging
    t_error = t_theta - t_averageHeading; // Getting error between distance of target and robot
    t_integral += t_error; // Adding area (integral) between each iteration
    //pros::lcd::print(5, "turn error: %f ", t_error); // Debugging
    //pros::lcd::print(7, "Target theta: %f ", t_theta); // Debugging

    // In case we make it to the setpoint or overshoot the target reset integral since we no longer need the extra power
    if (t_error == 0 || t_error > t_theta) {
      t_integral = 0;
    }

    t_derivative = t_error - t_previouserror; // Calculating the rate of change in error 
    t_previouserror = t_error;

    double voltage = ((t_error * t_kp) + (t_integral * t_ki) + (t_derivative * t_kd)) * 94; // Merging all calculations into final voltage power
    //pros::lcd::print(4, "error: %f ", t_error); // Debugging

    utility::leftvreq(voltage); // Making motors move amount in volts
    utility::rightvreq(voltage * -1); // Making motors move amount in volts

    if(fabs(t_error) < t_tolerance){
      t_threshholdcounter++;
    }
    else{
      t_threshholdcounter = 0;
    }
    if (t_threshholdcounter > 7){
      utility::stop();
      //pros::lcd::print(6, "Broke out: %f "); // Debugging
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


double radiusSearch(double xPoint, double yPoint)
{
  double xPos = xPoint - gx;
  double yPos = yPoint - gy;

  double halfwayX = xPos / 2;
  double halfwayY = yPos / 2;

  double intersectionLineSlope = (yPoint - gy) / (xPoint - gx);
  double intersectionLineb = 0;

  double halfwaySlope = 1 / (intersectionLineSlope * -1);
  double halfwayb = halfwayY - halfwaySlope * halfwayX;
  double radius = -1 * halfwayb / halfwaySlope;

  return fabs(radius);
}

void ArcPID(double targetX, double targetY){

  double startError = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));

  const double a_kP              = 15;
  const double a_driveMultiplier = 95;

  double a_previousTurnAngle = 0;
  double a_distanceError = startError;
  double a_previousError = startError;
  double a_failsafeCheck = 0;

  int a_failsafeCounter = 0;
  int a_breakCounter = 0;

  bool a_turnFixToggle = false;
  bool a_rightTurn = false;
  bool a_switched = false;

  while (true){
    SecondOdometry();
    a_distanceError = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));
    double speed = a_kP * a_distanceError * a_driveMultiplier;
    double modifier = 0.5;

    if (fabs(speed) > 12000)
    {
      speed = 12000;
    }
    if (a_failsafeCounter % 50 == 0){
      if (abs(abs(a_failsafeCheck)- abs(a_distanceError)) < 0.5){
        pros::lcd::print(2, "Broke out");
        break;
      }
      a_failsafeCheck = a_distanceError;
    }

    double targetTheta = atan2f(targetX - gx, targetY - gy);
    targetTheta = (targetTheta - ImuMon() * M_PI / 180);
    targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;

    if (targetTheta >= 0){
      a_rightTurn = true; // turn right
    }
    else{
      a_rightTurn = false; // turn left
    }

    if (fabs(targetTheta) < 1.5) // Close enough to theta just drive lmao
    { 
      utility::leftvreq(speed);
      utility::rightvreq(speed);
    }
    else if (a_rightTurn) // Turning right
    {
      utility::leftvreq(speed);
      utility::rightvreq(speed * modifier);
    }
    else // Turning left
    {
      utility::leftvreq(speed * modifier);
      utility::rightvreq(speed);
    }

    if (fabs(a_distanceError) < 4){
      utility::stop();
      break;
    }

    a_failsafeCounter++;
    a_previousError = a_distanceError;
    pros::delay(10);
  }
}