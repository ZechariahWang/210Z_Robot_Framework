// LIBRARIES
#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"


// Helper functions
int PointToPointDist(std::array<double, 2> pt1, std::array<double, 2> pt2){
  double distance = sqrt(pow(pt2[0] - pt1[0], 2) + pow(pt2[1] - pt1[1], 2));
  return distance;
}

// Returns 1 if num is + and -1 if -
int numStat(int num){
  if (num >= 0){
    return 1;
  }
  else{
    return -1;
  }
}

// Find min angle between target heading and current heading
int find_min_angle(int targetHeading, int currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){
    turnAngle = turnAngle - (utility::sgn(turnAngle) * 360);
  }

  return turnAngle;
}

// Radian -> Degree helper function
int radian_to_degrees(double angle){
  return angle * 180 / M_PI;
}

// Degree -> Radian helper function
int degrees_to_radians(double angle){
  return angle * M_PI / 180;
}

double kp_lin_M = 7;
double kp_turn_M  = 3;

// This function is solely for movement in Pure Pursuit.
void MotionAlgorithms::GTP_Movement(double target_X, double target_Y){
  SecondOdometry();
  double currentX = gx;
  double currentY = gy;
  double targetX = target_X;
  double targetY = target_Y;

  double abstargetAngle = atan2f(targetX - gx, targetY - gy) * 180 / M_PI;

  if (abstargetAngle < 0){
    abstargetAngle += 360;
  }

  double D = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
  double errorTerm1 = find_min_angle(abstargetAngle, ImuMon());
  double turn_Error = errorTerm1;

  if (turn_Error > 180 || turn_Error < -180){
    turn_Error = turn_Error - (utility::sgn(turn_Error) * 360);
  }

  int linearVel = 150;
  int turnVel = kp_turn_M * turn_Error;

  // if (abs(linearVel) > (350 - abs(turnVel))){
  //   linearVel = 350 - abs(turnVel);
  // }

  int leftVel_f = linearVel + turnVel;
  int rightVel_f = linearVel - turnVel;

  utility::leftvelreq(leftVel_f);
  utility::rightvelreq(rightVel_f);

}

double kp_h = 2;
double kp_hl = 10;

// This function should move to a point by calculating the turn error relative to the target
void MotionAlgorithms::NHMTP(double target_X, double target_Y){
  while (true){
    SecondOdometry();
    double absTargetAngle_h = atan2f(target_X - gx, target_Y - gy) * 180 / M_PI;
    double distance = sqrt(pow(target_X - gx, 2) + pow(target_Y - gy, 2));

    if (absTargetAngle_h < 0){
      absTargetAngle_h += 360;
    }
    // pros::lcd::print(5, "target angle: %d", absTargetAngle_h);
    double turnError_h = absTargetAngle_h - ImuMon();
    if (turnError_h > 180 || turnError_h < -180){
      turnError_h = turnError_h - (utility::sgn(turnError_h) * 360);
      // turnError_h = -1 * utility::sgn(turnError_h) * (360 - fabs(turnError_h));
    }

    double turnVel = kp_h * turnError_h;
    double linVel = 155;

    int leftVel_h = linVel + turnVel;
    int rightVel_h = linVel - turnVel;

    if (sqrt(pow(target_X - gx, 2) + pow(target_Y - gy, 2)) < 3){
      utility::leftvelreq(0);
      utility::rightvelreq(0);
      break;
    }

    utility::leftvelreq(leftVel_h);
    utility::rightvelreq(rightVel_h);

    pros::delay(10);

  }
}

double targetTolerance = 5;
double finalLocTolerance = 5;
double kp_lin = 13;
double kp_turn = 3.2;

// Move to reference pose algorithm
void MotionAlgorithms::MTRP(double tx, double ty, double targetHeading, double GlobalHeading){
  MotionAlgorithms Auton_Framework;
  FinalizeAuton data;
  eclipse_PID pid;
  while (true){
    SecondOdometry();
    data.DisplayData();

    double currentX = gx;
    double currentY = gy;
    double targetX = tx;
    double targetY = ty;

    double abstargetAngle = atan2f(targetX - gx, targetY - gy) * 180 / M_PI;

    if (abstargetAngle < 0){
      abstargetAngle += 360;
    }

    double D = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
    double alpha = find_min_angle(abstargetAngle, targetHeading);
    double errorTerm1 = find_min_angle(abstargetAngle, ImuMon());

    double beta = atan(1/ D) * 180 / M_PI;
    double turn_Error;

    if (alpha < 0){
      beta = -beta;
    }

    if (fabs(alpha) < fabs(beta)){
      turn_Error = errorTerm1 + alpha;
    }
    else{
      turn_Error = errorTerm1 + beta;
    }

    if (turn_Error > 180 || turn_Error < -180){
      turn_Error = turn_Error - (utility::sgn(turn_Error) * 360);
    }

    int linearVel = kp_lin * D;
    int turnVel = kp_turn * turn_Error;

    double closetoTarget = false;

    if (D < targetTolerance){
      closetoTarget = true;
    }
    if (closetoTarget){
      linearVel = kp_lin * D * utility::sgn(cos(turn_Error * M_PI / 180));
      turn_Error = find_min_angle(targetHeading, ImuMon());
      turnVel = kp_turn * atan(tan(turn_Error * M_PI / 180)) * 180 / M_PI;
    }

    if (abs(linearVel) > (350 - abs(turnVel))){
      linearVel = 350 - abs(turnVel);
    }

    int leftVel_f = linearVel + turnVel;
    int rightVel_f = linearVel - turnVel;
    int linError_f = sqrt(pow(tx - gx, 2) + pow(ty - gy, 2));

    utility::leftvelreq(leftVel_f);
    utility::rightvelreq(rightVel_f);

    if ((fabs(targetX - gx) < finalLocTolerance) && (fabs(targetY - gy) < finalLocTolerance)){
      utility::leftvelreq(0);
      utility::rightvelreq(0);
      Auton_Framework.TurnPID(targetHeading);
      break;
    }

    std::cout << "current x " << gx << std::endl;
    std::cout << "current y " << gy << std::endl;

    pros::delay(10);

  }
}

// Move to reference pose algorithm
void MotionAlgorithms::MTRP_Movement(double tx, double ty, double targetHeading, double GlobalHeading){

  MotionAlgorithms Auton_Framework;
  SecondOdometry();

  double currentX = gx;
  double currentY = gy;
  double targetX = tx;
  double targetY = ty;

  double abstargetAngle = atan2f(targetX - gx, targetY - gy) * 180 / M_PI;

  if (abstargetAngle < 0){
    abstargetAngle += 360;
  }

  double D = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
  double alpha = find_min_angle(abstargetAngle, targetHeading);
  double errorTerm1 = find_min_angle(abstargetAngle, ImuMon());

  double beta = atan(1/ D) * 180 / M_PI;
  double turn_Error;

  if (alpha < 0){
    beta = -beta;
  }

  if (fabs(alpha) < fabs(beta)){
    turn_Error = errorTerm1 + alpha;
  }
  else{
    turn_Error = errorTerm1 + beta;
  }

  if (turn_Error > 180 || turn_Error < -180){
    turn_Error = turn_Error - (utility::sgn(turn_Error) * 360);
  }

  int linearVel = kp_lin * D;
  int turnVel = kp_turn * turn_Error;

  double closetoTarget = false;

  if (D < targetTolerance){
    closetoTarget = true;
  }
  if (closetoTarget){
    linearVel = kp_lin * D * utility::sgn(cos(turn_Error * M_PI / 180));
    turn_Error = find_min_angle(targetHeading, ImuMon());
    turnVel = kp_turn * atan(tan(turn_Error * M_PI / 180)) * 180 / M_PI;
  }

  if (abs(linearVel) > (300 - abs(turnVel))){
    linearVel = 300 - abs(turnVel);
  }

  int leftVel_f = linearVel + turnVel;
  int rightVel_f = linearVel - turnVel;
  int linError_f = sqrt(pow(tx - gx, 2) + pow(ty - gy, 2));

  utility::leftvelreq(leftVel_f);
  utility::rightvelreq(rightVel_f);

}

// Turn to target coordinate position
void MotionAlgorithms::TurnToPoint(int targetX, int targetY){
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
  TurnPID(0 + angle);
}

double p_deltaX = 0;
double p_deltaY = 0;
double previousDriveError = 0;
double previousTurnError = 0;
double driveSlewOutput = 0;
double turnSlewOutput = 0;
double previousSlewTurn = 0;
double deltaSlewTurn = 0;

double d_kp = 13;
double d_kd = 1.3;

double t_kp = 1.6;
double t_kd = 1.5;

double p_tolerance = 0.8;
double angleTolerance = 3;

// Go to point function. Same concept as above, but for mecanum drives and holomonics.
void MotionAlgorithms::GoToCoordPos(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate){

  while (true){
    
    SecondOdometry();

    targetTheta = targetTheta;
    double theta = ImuMon() * M_PI / 180;
    double driveError = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));
    double positionHypo = sqrt(pow(gx,2) + pow(gy, 2));
    double derivative = ((gx * d_deltaX) + (gy * d_deltaY)) / positionHypo;
    double driveOutput = (driveError * d_kp) + ((driveError - previousDriveError) * d_kd);

    double turnError = (-theta - targetTheta);
    double turnErrorRad = turnError * M_PI /180;
    turnError = atan2f(sinf(turnError), cosf(turnError));
    double turnOutput = Turn_PID(targetTheta);

    double angleDesired = atan2f(targetX - gx, targetY - gy);
    double angleDrive = (angleDesired - theta);
    angleDrive = atan2f(sinf(angleDrive), cosf(angleDrive));

    double velDrive = driveOutput * cos(angleDrive); 
    double velStrafe = driveOutput * sin(angleDrive);

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    if(fabs(driveError) < p_tolerance && fabs(turnError) < 3){
      utility::leftvelreq(0);
      utility::rightvelreq(0);
      break;
    }

    if(fabs(driveError) < p_tolerance && fabs(turnError) < angleTolerance){
      utility::leftvelreq(0);
      utility::rightvelreq(0);
      break;
    } 
    else{
      speedFL = velDrive - velStrafe + turnOutput;
      speedFR = velDrive + velStrafe - turnOutput;
      speedBL = velDrive + velStrafe + turnOutput;
      speedBR = velDrive - velStrafe - turnOutput;
    }

    DriveFrontLeft.move_velocity((speedFL));
    DriveFrontRight.move_velocity((speedFR));
    DriveBackLeft.move_velocity((speedBL));
    DriveBackRight.move_velocity((speedBR));

    previousSlewTurn = turnSlewOutput;
    previousTurnError = turnError;
    previousDriveError = driveError;

    pros::delay(10);

  }
}

int lastFoundIndex_G = 0;
double lookAheadDistance = 0.8;
double lastFoundIndex = 0;
double linearVel = 127;

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

  // pros::lcd::print(5, "running");

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

  const double PP_KP_M = 3;
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

  //return GoalPoint, lastFoundIndex, turnVel;

  return 1;
  // hi

}

void MotionAlgorithms::PurePursuitRunner(std::vector<std::array<double, 2>> Path){
  int linearVel = 100;
  int finalPoint = 2;

  while (true){

    double  turnVelocity = SecondPurePursuit(Path);
    double leftmotor =  linearVel - turnVelocity;
    double rightmotor = linearVel + turnVelocity;

    DriveFrontLeft.move_velocity(leftmotor);
    DriveFrontRight.move_velocity(rightmotor);
    DriveBackLeft.move_velocity(leftmotor);
    DriveBackRight.move_velocity(rightmotor);

    if (lastFoundIndex == finalPoint){
      DriveFrontLeft.move_velocity(0);
      DriveFrontRight.move_velocity(0);
      DriveBackLeft.move_velocity(0);
      DriveBackRight.move_velocity(0);

      break;
    }
  }
}



