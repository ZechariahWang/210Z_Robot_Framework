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

  return GoalPoint, lastFoundIndex, turnVel;
}

void MotionAlgorithms::PurePursuitRunner(std::vector<std::array<double, 2>> Path){
  int linearVel = 100;
  int finalPoint = 2;
  while (true){
    double goalpoint, LFindex, turnVelocity = SecondPurePursuit(Path);

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


////////////////////////////////////////////////*/
/* Section: Turn To Point  
///////////////////////////////////////////////*/

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
  pros::lcd::print(3, "theta: %f ", angle);
  TurnPID(0 + angle);
  
  pros::lcd::print(4, "distance X: %f ", distanceX);
  pros::lcd::print(5, "distance Y: %f ", distanceY);
  pros::lcd::print(6, "TTP sequence finished, exiting control.");
}

////////////////////////////////////////////////*/
/* Section: GTC (FOR HOLOMONIC ONLY)
///////////////////////////////////////////////*/

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

    pros::lcd::print(4, "drive output: %.2f tt %f ", driveOutput, targetTheta);
    pros::lcd::print(5, "turn output: %f t: %f", turnOutput,theta);
    pros::lcd::print(6, "DE: %f TE %fm", driveError, turnError * (180 / M_PI));

    double velDrive = driveOutput * cos(angleDrive); 
    double velStrafe = driveOutput * sin(angleDrive);

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    if(fabs(driveError) < p_tolerance && fabs(turnError) < 3){
      DriveFrontLeft.move_velocity(0);
      DriveFrontRight.move_velocity(0);
      DriveBackLeft.move_velocity(0);
      DriveBackRight.move_velocity(0);
      pros::lcd::print(7, "broke out 1");
      break;
    }

    if(fabs(driveError) < p_tolerance && fabs(turnError) < angleTolerance){
      DriveFrontLeft.move_velocity(0);
      DriveFrontRight.move_velocity(0);
      DriveBackLeft.move_velocity(0);
      DriveBackRight.move_velocity(0);
      pros::lcd::print(7, "broke out 2");
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

double targetHeading = 225;
double targetTolerance = 0.1;
double finalLocTolerance = 2;
double kp_lin = 30;
double kp_turn = 1.7;

int find_min_angle(int targetHeading, int currentrobotHeading){
  double turnAngle = targetHeading - currentHeading;
  if (turnAngle > 180 || turnAngle < -180){
    turnAngle = -1 * utility::sgn(turnAngle) * (360 - fabs(turnAngle));
  }

  return turnAngle;
}

int NonHolomonicGoToPoint(double t_x, double t_y){
  double currentX = gx;
  double currentY = gy;

  double targetX = t_x;
  double targetY = t_y;

  double abstargetAngle = atan2f((targetY - currentY), (targetX - currentX)) * 180 / M_PI;

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

  if (abs(alpha) < abs(beta)){
    turn_Error = errorTerm1 + alpha;
  }
  else{
    turn_Error = errorTerm1 + beta;
  }

  double linearVel = kp_lin * D;
  double turnVel = kp_turn * turn_Error;

  double closetoTarget = false;

  if (D < targetTolerance){
    closetoTarget = true;
  }
  if (closetoTarget){
    linearVel = kp_lin * D * utility::sgn(cos(turn_Error * M_PI / 180));
    turn_Error = find_min_angle(targetHeading, currentHeading);
    turnVel = kp_turn * atan(tan(turn_Error * M_PI / 180)) * 180 / M_PI;
  }

  if (abs(linearVel) > 70){
    linearVel = utility::sgn(linearVel) * 70;
  }
  if (turnVel > (100 - abs(turnVel))){
    linearVel = 100 - abs(turnVel);
  }

  return linearVel, turnVel;
}

void MotionAlgorithms::MTRP(double tx, double ty){
  while (true){
    double linearVel_NH, turnVel_NH = NonHolomonicGoToPoint(tx, ty);
    double leftVel = linearVel_NH - turnVel_NH;
    double rightVel = linearVel_NH + turnVel_NH;

    double linError = sqrt(pow(tx - gx, 2) + pow(ty - gy, 2));

    if (linError < finalLocTolerance){
    
      DriveFrontLeft.move_velocity(0);
      DriveFrontRight.move_velocity(0);
      DriveBackLeft.move_velocity(0);
      DriveBackRight.move_velocity(0);
      break;
    }
    
    DriveFrontLeft.move_velocity(leftVel);
    DriveFrontRight.move_velocity(rightVel);
    DriveBackLeft.move_velocity(leftVel);
    DriveBackRight.move_velocity(rightVel);
  }
}
