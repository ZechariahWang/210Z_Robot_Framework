// LIBRARIES
#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"


int PointToPointDist(std::array<double, 2> pt1, std::array<double, 2> pt2){
  double distance = sqrt(pow(pt2[0] - pt1[0], 2) + pow(pt2[1] - pt1[1], 2));
  return distance;
}

int numStat(int num){
  if (num >= 0){
    return 1;
  }
  else{
    return -1;
  }
}

// frc is homosexual
void RamseteController(){
  double desiredX = gx;
  double desiredY = gy;
  double desiredTheta = ImuMon();

  
}

int lastFoundIndex_G = 0;
double lookAheadDistance = 2;
double lastFoundIndex = 0;
double linearVel = 12000;

bool using_rotation = false;

double currentX;
double currentY;
double currentHeading;
std::array<double, 2> GoalPoint;
std::array<double, 2> CurrentPosition;

// pros lied to me
int SecondPurePursuit(std::vector<std::array<double, 2>> Path){
  
  SecondOdometry();

  currentX = gx;
  currentY = gy;
  CurrentPosition[0] = gx;
  CurrentPosition[1] = gy;
  currentHeading = ImuMon();

  pros::lcd::print(5, "running");

  // lastFoundIndex = lastFoundIndex_G;
  bool IntersectionFound = false;
  bool startingIndex = lastFoundIndex;

  for (int i = startingIndex; i < Path.size() - 1; i++){
    double x1 = Path[i][0] - currentX;
    double y1 = Path[i][1] - currentY;
    double x2 = Path[i + 1][0] - currentX;
    double y2 = Path[i + 1][1] - currentY;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = sqrt(pow(dx, 2) + pow(dy, 2));
    double d = x1 * y2 - x2 * y1;

    double discriminant = (pow(lookAheadDistance, 2)) * (pow(dr, 2)) - pow(d, 2);

    if (discriminant >= 0){
      double sol_x1 = (d * dy + numStat(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
      double sol_x2 = (d * dy - numStat(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
      double sol_y1 = (-d * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
      double sol_y2 = (-d * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);

      std::array<double, 2> sol_pt1 = {sol_x1 + currentX, sol_y1 + currentY};
      std::array<double, 2> sol_pt2 = {sol_x2 + currentX, sol_y2 + currentY};

      double minX = std::min(Path[i][0], Path[i+1][0]);
      double minY = std::min(Path[i][1], Path[i+1][1]);
      double maxX = std::max(Path[i][0], Path[i+1][0]);
      double maxY = std::max(Path[i][1], Path[i+1][1]);

      if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)) || ((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))) {
        IntersectionFound = true;

        if (((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)) && ((minX <= sol_pt2[0] <= maxX) && (minY <= sol_pt2[1] <= maxY))){
          if (PointToPointDist(sol_pt1, Path[i + 1]) < PointToPointDist(sol_pt2, Path[i + 1])){
            GoalPoint = sol_pt1;
            
          }
          else{
            GoalPoint = sol_pt2;
          }

        }
        else{
          if ((minX <= sol_pt1[0] <= maxX) && (minY <= sol_pt1[1] <= maxY)){
            GoalPoint = sol_pt1;
          }
          else{
            GoalPoint = sol_pt2;

          }
        }
        if (PointToPointDist(GoalPoint, Path[i + 1]) < PointToPointDist({currentX, currentY}, Path[i + 1])){
          lastFoundIndex = i;
          break;
        }
        else{
          lastFoundIndex = i + 1;
        }
      }
      else{
        IntersectionFound = false;
        GoalPoint = {Path[lastFoundIndex][0], Path[lastFoundIndex][1]};
      }
    }
  }

  const double PP_KP_M = 1000;
  const double PP_KP_T = 1000;

  double linearError = sqrt(pow(GoalPoint[1] - currentY, 2) + pow((GoalPoint[0] - currentX), 2));
  double absTargetAngle = atan2f(GoalPoint[1] - currentY, GoalPoint[0] - currentX) * 180 / M_PI;

  if (absTargetAngle < 0){
    absTargetAngle += 360;
  }

  double turnError = absTargetAngle - currentHeading;

  if ((turnError > 180) || (turnError < -180)){
    turnError = -1 * numStat(turnError) * (360 - fabs(turnError));
  }

  double turnVel = PP_KP_T * turnError; 
  double linearVel = PP_KP_M * linearError;

  double leftmotor =  linearVel - turnVel;
  double rightmotor = linearVel + turnVel;

  const int multiplier = 34;

  DriveFrontLeft.move_voltage(leftmotor * multiplier);
  DriveBackLeft.move_voltage(leftmotor * multiplier);
  DriveFrontRight.move_voltage(rightmotor * multiplier);
  DriveBackRight.move_voltage(rightmotor * multiplier);

  pros::lcd::print(4, "lin err: %f ", linearError);
  pros::lcd::print(5, "turn err: : %f ", turnError);

  return 0;
}

void PurePursuitRunner(std::vector<std::array<double, 2>> Path){

  std::array<double, 2> GoalPoint;
  int lastFoundIndex;
  double turnVelnew;

  // while (true){
    SecondPurePursuit(Path);
    // pros::delay(10);
  //}

  // MoveToPoint_PP(GoalPoint, lastFoundIndex, turnVelnew);
}


////////////////////////////////////////////////*/
/* Section: Turn To Point  
///////////////////////////////////////////////*/

void TurnToPoint(int targetX, int targetY){

  SecondOdometry();
  

  double finalAngle;

  double distanceX = targetX - gx;
  double distanceY = targetY - gy;
 
  double hypot = pow(distanceX, 2) + pow(distanceY, 2);
  double targetDistance = sqrt(hypot);
  double robotHeading = ImuMon();
  double ACTUALROBOTHEADING = imu_sensor.get_rotation();
  double resetAmount = robotHeading;

  if (resetAmount < 180){
    finalAngle = resetAmount;
  }

  double angle = atan2f(distanceX, distanceY) * 180 / M_PI;
  pros::lcd::print(3, "theta: %f ", angle);
  TurnPID(0 + angle);
  

  pros::lcd::print(4, "distance X: %f ", distanceX);
  pros::lcd::print(5, "distance Y: %f ", distanceY);
  pros::lcd::print(6, "TTP sequence finished, exiting control.");

}

double p_deltaX = 0;
double p_deltaY = 0;
double previousDriveError = 0;
double previousTurnError = 0;
double driveSlewOutput = 0;
double turnSlewOutput = 0;
double previousSlewTurn = 0;
double deltaSlewTurn = 0;

double d_kp = 14;
double d_kd = 0.82;

double t_kp = 20;
double t_kd = 1.5;

double turnRate = 5;
double driveRate = 5;
double p_tolerance = 5;
double angleTolerance = 5;



void GoToCoordPos(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed){

  while (true){

    SecondOdometry();

    double driveError = sqrt(pow(targetX - gx, 2) + pow(targetY - gy,2));
    double positionHypo = sqrt(pow(gx,2) + pow(gy, 2));
    double derivative = ((gx * p_deltaX) + (gy * p_deltaY)) / positionHypo;
    double driveOutput = (driveError * d_kp) - ((driveError - previousDriveError) * d_kd);

    double turnError = (-ImuMon() - targetTheta);
    turnError = atan2f(sin(turnError), cos(turnError));
    double turnOutput = (turnError * t_kp ) + ((turnError - previousTurnError) * t_kd);

    double angleDesired = atan2f(targetX - gx, targetY - gy);
    double angleDrive = (angleDesired - ImuMon());
    angleDrive = atan2f(sinf(angleDrive), cosf(angleDrive));

    double velDrive = cos(angleDrive);
    double velStrafe = sin(angleDrive);

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    if(fabs(driveError) < 3 && fabs(turnError * (180 / M_PI)) < 2){
      break;
    }

    if(fabs(driveError) < p_tolerance && fabs(turnError * (180 / M_PI)) < angleTolerance){
      speedFR = 0;
      speedFL = 0;
      speedBR = 0;
      speedBL = 0;
      break;
    } 
    else{
      speedFL = -velDrive - velStrafe * 95;
      speedFR = velDrive  - velStrafe * 95;
      speedBL = -velDrive + velStrafe * 95;
      speedBR = velDrive + velStrafe * 95;
    }

    DriveFrontLeft.move_voltage(speedFL * 95);
    DriveFrontRight.move_voltage(speedFR * 95);
    DriveBackLeft.move_voltage(speedBL * 95);
    DriveBackRight.move_voltage(speedBR * 95);


    previousTurnError = turnError;
    previousDriveError = driveError;

    pros::delay(10);

    
  }
}

