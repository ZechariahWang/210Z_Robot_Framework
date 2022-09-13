#include "main.h"
#include "vector"
#include "variant"
#include "array"


////////////////////////////////////////////////*/
/* Section: Primary Odometry Variable Declaration
///////////////////////////////////////////////*/

double localthetaPrimary                           = 0;
double rotationcounter                             = 0;

double currentleft                                 = 0;
double currentright                                = 0;
double currentcenter                               = 0;
double currentTheta                                = 0;

double encodertheta                                = 0;
double totalencodertheta                           = 0;

double deltaLeft                                   = 0;
double deltaRight                                  = 0;
double deltaCenter                                 = 0;
double deltaTheta                                  = 0;

double deltatheory                                 = 0;
double theory                                      = 0;
double totalEncoderTheta                           = 0;

double deltaX                                      = 0;
double deltaY                                      = 0;

double previousleft                                = 0;
double previousright                               = 0;
double previouscenter                              = 0;
double previoustheta                               = 0;

double global_x                                    = 0;
double global_y                                    = 0;
double global_theta                                = 0;

////////////////////////////////////////////////*/
/* Section: SS Odometry Variable Declaration
///////////////////////////////////////////////*/

double CL                                          = 0; // Current Left
double CR                                          = 0; // Current Right
double CC                                          = 0; // Current Center
double RT                                          = 0; // Rotation Theta

double LL                                          = 0; // Last Left
double LR                                          = 0; // Last Right
double LC                                          = 0; // Last Center
double LT                                          = 0; // Last Theta

double DL                                          = 0; // Delta Left
double DR                                          = 0; // Delta Right
double DC                                          = 0; // Delta Center
double DT                                          = 0; // Delta Theta
double Theory                                      = 0; // Theory
double DTheory                                     = 0; // Delta Theory
double TRT                                         = 0; // Total Rotation Theta


double DX                                          = 0; // Delta X
double DY                                          = 0; // Delta Y

double x                                           = 0; // Local Position X
double y                                           = 0; // Local Position Y
double h                                           = 0; // Theta Guidance

double px                                          = 0; // Global Position X
double py                                          = 0; // Global Position Y
double localencodertheta                           = 0; // Local Position Theta
double pt                                          = 0; // Global Position Theta (Heading)

double counter                                     = 0; // Counter

////////////////////////////////////////////////*/
/* Section: DS Odometry Variable Declaration
///////////////////////////////////////////////*/

double DS_CF                                       = 0; // Current forwards
double DS_CC                                       = 0; // Current center
double DS_COT                                      = 0; // Current O theta
double DS_RT                                       = 0; // Rotation theta

double DS_DF                                       = 0; // Delta forwards_
double DS_DC                                       = 0; // Delta Center
double DS_DT                                       = 0; // Delta theta
double DS_DOT                                      = 0; // Delta O theta

double DS_D_T                                      = 0; // Delta Theory
double DS_D_T2                                     = 0; // Delta Theory 2

double DS__T                                       = 0; // Theory
double DS__T2                                      = 0; // Theory 2
double DS_TRT                                      = 0; // Total Rotation Theta

double DS_PF                                       = 0; // Previous forwards
double DS_PC                                       = 0; // Previous center
double DS_POT                                      = 0; // Previous O theta
double DS_previoustheta                            = 0; // previous theta
double localtheta                                  = 0;

double DS_DX                                       = 0; // Delta X
double DS_DY                                       = 0; // Delta Y

double gx                                          = 0; // Global X
double gy                                          = 0; // Global Y
double gh                                          = 0; // Global H

////////////////////////////////////////////////*/
/* Section: Two Wheel Odometry Variables
///////////////////////////////////////////////*/

double deltaArcLength                              = 0;
double previousArcLength                           = 0;
double currentarclength                            = 0;

double d_currentForward                            = 0;
double d_currentCenter                             = 0;
double d_currentOtheta                             = 0;
double d_rotationTheta                             = 0;

double d_deltaForward                              = 0;
double d_deltaCenter                               = 0;
double d_deltaTheta                                = 0;
double d_deltaOTheta                               = 0;

double d_deltaTheory                               = 0;
double d_deltaTheory2                              = 0;
double d_Theory                                    = 0;
double d_Theory2                                   = 0;
double d_totalRotationTheta                        = 0;

double d_deltaX                                    = 0;
double d_deltaY                                    = 0;

double d_previousForward                           = 0;
double d_previousCenter                            = 0;
double d_previousOTheta                            = 0;
double d_previoustheta                             = 0;

// Util function. Limit IMU to 360 degrees
double Global::ImuMonitor() {
  double theta = imu_sensor.get_rotation();
 
  while (theta < 0) {
    theta += 360;
  }
  while (theta > 360) {
    theta = 0;
  }
 
  return theta;
 
}

double ImuMon() {
  double theta = fmod(imu_sensor.get_rotation(), 360);
 
  while (theta < 0) {
    theta += 360;
  }
  while (theta > 360) {
    theta -= 360;
  }
 
  return theta;
 
}

// This function is for the primary odom framework used within the robot.
void SecondOdometry() {

  Global OdomUtil;
  pros::Mutex mutex;

  mutex.take(10);

  double theta = imu_sensor.get_rotation();
  double RX = (cos(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // Local X value
  double RY = (sin(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // local Y value

  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(RY, RX) + M_PI); // theta is in radians
    double localtheta = theta * 58.5; // Translated value relative to IMU values
 
    if (localtheta > 361 && localtheta < 368) {
      std::cout << "In danger zone" << std::endl; // theta values here are not being monitored
    }
    else {
      std::cout << "all g" << std::endl; // its not dead lessgo
    }
    localtheta = theta; // Updating translated theta value
  }

  double r = 29 / (2 * M_PI);
  double angleRadian = imu_sensor.get_rotation() * (M_PI / 180);
  currentarclength = angleRadian * r;

  double val = imu_sensor.get_rotation();
  double offset = (2 * val * 6) / 2.75;

  d_currentForward = (DriveFrontLeft.get_position() * M_PI / 180);
  d_currentCenter = ((RotationSensor.get_position() * 3 / 500) * M_PI / 180);
  double imuval = imu_sensor.get_rotation();
  d_currentOtheta = theta;
  d_rotationTheta = ((DL - DR) / 14.375); // In case of no inertial, we can use encoders instead

  d_deltaForward = d_currentForward - d_previousForward;
  d_deltaCenter = d_currentCenter - d_previousCenter;
  d_deltaTheta = theta - d_previoustheta;
  d_deltaOTheta = d_currentOtheta - d_previousOTheta;
  deltaArcLength = currentarclength - previousArcLength;

  d_deltaTheory = d_deltaOTheta;
  d_deltaTheory2 = d_deltaOTheta;
  d_Theory += d_deltaTheory;
  d_Theory2 += d_deltaTheory2;
  d_totalRotationTheta += d_rotationTheta;

  d_deltaX = (((d_deltaForward) * 1 * -sin(-theta)) - ((d_deltaCenter - deltatheory) * 1 * -cos(-theta))); 
  d_deltaY = (((d_deltaForward) * 1 * cos(-theta)) - ((d_deltaCenter - deltatheory) * 1 * -sin(-theta)));

  gx = gx + d_deltaX;
  gy = gy + d_deltaY;

  if (gx < -99999999999999){
    gx = 0;
  }

  d_previousForward = d_currentForward;
  d_previousCenter = d_currentCenter;
  d_previousOTheta = d_currentOtheta;
  d_previoustheta = theta;
  previousArcLength = currentarclength;


  pros::lcd::print(1, "X: %f ", gx);
  pros::lcd::print(2, "Y: %f ", gy);
  //pros::lcd::print(4, "Forward: %f ", d_currentForward);
  //pros::lcd::print(5, "Theory 2: %f ", d_Theory2);
  // pros::lcd::print(5, "Arc length: %f ", currentarclength);
  pros::lcd::print(3, "theta: %f", ImuMon());
  // pros::lcd::print(7, "imu: %f", imu_sensor.get_rotation());
  //pros::lcd::print(7, "df: %f", d_deltaForward);

  mutex.give();

}

void Odometry::StandardOdom() {

  Global OdomUtil;
 
  double RX = (cos(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // Local X value
  double RY = (sin(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // local Y value
 
  if (fmod(counter, 3) < 1) {
 
    pt = std::abs(atan2f(RY, RX) + M_PI); // Global Theta value
    localencodertheta = pt * 58.5; // Translated value relative to IMU values
 
    if (localencodertheta > 361 && localencodertheta < 368) {
      std::cout << "In danger zone" << std::endl; // theta values here are not being monitored
    }
    else {
      std::cout << "all g" << std::endl; // we good
    }
 
    localencodertheta = pt; // Updating translated theta value

  }
 
  CL = DriveFrontLeft.get_position() * M_PI / 180; // Getting the current left wheel value
  CR = DriveFrontRight.get_position() * M_PI / 180; // Getting the current right wheel value
  CC = FrontAux.get_value() * M_PI / 180; // Getting the current center wheel value
  RT = ((DL - DR) / 14.375); // Getting the local new robot rotation value
 
 
  DL = CL - LL; // Delta Left value
  DR = CR - LR; // Delta Right value
  DC = CC - LC; // Delta Center value
  DT = pt - LT; // Delta Theta value
 
  DT = (DL - DR) * M_PI / 180;
  Theory += DT;
  TRT += RT; // Updating total rotation theta amount
 
  DX = (((DL + DR) / 2 * 1 * -sin(-pt)) - ((DC + DT) * 1 * -cos(-pt))); // funnie math 
  DY = (((DL + DR) / 2 * 1 * cos(-pt)) - ((DC + DT) * 1 * -sin(-pt))); // :)
 
 
  px += DX; // Updating global X value
  py += DY; // Updating global Y value
 
  LL = CL; // Updating old left wheel values
  LR = CR; // Updating old right wheel values
  LC = CC; // Updating old center wheel values
  LT = pt; // Updating old theta values


  pros::lcd::print(1, "X: %f ", px);
  pros::lcd::print(2, "Y: %f ", py);

}

// This function contains old methods for 2 wheel odom
// Added some arc length theory later on but for the most part its the original code
void Odometry::SecondOdometryOLD() {

  Global OdomUtil;

  double theta = (imu_sensor.get_rotation() + imu_sensor_secondary.get_rotation()) / 2;

  double RX = (cos(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // Local X value
  double RY = (sin(OdomUtil.ImuMonitor() * M_PI / 180 + M_PI)); // local Y value

  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(RY, RX) + M_PI); // theta is in radians
  }

  double r = 29 / (2 * M_PI);
  double angleRadian = imu_sensor.get_rotation() * (M_PI / 180);
  currentarclength = angleRadian * r;

  DS_CF = DriveFrontLeft.get_position() * M_PI / 180;
  DS_CC = ((RotationSensor.get_position() / 100) * M_PI / 180);
  DS_COT = theta;
  DS_RT = ((DL - DR) / 14.375); // In case of no inertial, we can use encoders instead

  DS_DF = DS_CF - DS_PF;
  DS_DC = DS_CC - DS_PC;
  DS_DT = theta - DS_previoustheta;
  DS_DOT = DS_COT - DS_POT;
  deltaArcLength = currentarclength - previousArcLength;

  DS_D_T = DS_DOT;
  DS_D_T2 = DS_DOT;
  DS__T += DS_D_T;
  DS__T2 += DS_D_T2;
  DS_TRT += DS_RT;

  DS_DX = (((DS_DF - DS_D_T2) * 1 * -sin(-theta)) - ((DS_DC - deltaArcLength) * 1 * -cos(-theta))); 
  DS_DY = (((DS_DF - DS_D_T2) * 1 * cos(-theta)) - ((DS_DC - deltaArcLength) * 1 * -sin(-theta)));


  //gx = gx + DS_DX - arclength;
  gx = gx + DS_DX;
  gy = gy + DS_DY;

  if (gx < -99999999999999){
    gx = 0;
  }

  DS_PF = DS_CF;
  DS_PC = DS_CC;
  DS_POT = DS_COT;
  DS_previoustheta = theta;
  previousArcLength = currentarclength;


  pros::lcd::print(1, "X: %f ", gx);
  pros::lcd::print(2, "Y: %f ", gy);
  pros::lcd::print(3, "Center: %f ", DS_CC);
  pros::lcd::print(4, "Theory: %f ", DS__T);
  pros::lcd::print(5, "Arc length: %f ", currentarclength);
  pros::lcd::print(6, "rotation: %f", imu_sensor.get_rotation());
  pros::lcd::print(7, "dc - dal: %f", DS_DC - deltaArcLength);

}


