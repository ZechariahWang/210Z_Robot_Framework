#include "main.h"

const double SpeedCompensator = 0.45; // Adjusts speed 

// Wrap angle to 2 PI
int AngleWrap_C::AngleWrap(double angle){
    while (angle < -M_PI){
        angle += 2 * M_PI;
    }
    while (angle > M_PI){
        angle -= 2 * M_PI;
    }

    return angle;
}

// Function takes in 2 points, and checks the intersection status between both points
std::vector<Point> LineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
    if (abs(linePoint1.getY() - linePoint2.getY()) < 0.003){
        linePoint1.setY(linePoint2.getY() + 0.003);
    }
    if (abs(linePoint1.getX() - linePoint2.getX()) < 0.003){
        linePoint1.setX(linePoint2.getX() + 0.003);
    }

    double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());
    double b = (linePoint1.getY()) - m1 * (linePoint1.getX());

    double x1 = linePoint1.getX() - circleCenter.getX();
    double y1 = linePoint1.getY() - circleCenter.getY();

    double quadraticA = 1.0 + pow(m1, 2);
    double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);
    double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

    quadraticB = (-2 * gx) + (2.0 * m1 * b) - (2 * gy * m1);
    quadraticC = pow(gx, 2) + pow(b, 2) - (2 * b * gy) + pow(gy, 2) - pow(radius, 2);


    std::vector<Point> allPoints;

    double minX = linePoint1.getX() < linePoint2.getX() ? linePoint1.getX() : linePoint2.getX();
    double maxX = linePoint1.getY() < linePoint2.getY() ? linePoint1.getY() : linePoint2.getY();

    if (linePoint1.getX() < linePoint2.getX()){
        minX = linePoint1.getX();
        maxX = linePoint2.getX();
    }
    else{
        maxX = linePoint1.getX();
        minX = linePoint2.getX();
    }

    try
    {
        // Solution 1
        double xRoot1 = (-quadraticB + sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot1 = m1 * (xRoot1) + y1;

        // xRoot1 += circleCenter.getX();
        // yRoot1 += circleCenter.getY();

        if (xRoot1 > minX && xRoot1 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot1);
            newPoint.setY(yRoot1);
            allPoints.push_back(newPoint);
        }
        else{
            // No Points
        }

        // Solution 2
        double xRoot2 = (-quadraticB - sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot2 = m1 * (xRoot2) + y1;

        xRoot1 += circleCenter.getX();
        yRoot2 += circleCenter.getY();

        if (xRoot2 > minX && xRoot2 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot2);
            newPoint.setY(yRoot2);
            allPoints.push_back(newPoint);
        }
        else{
            // No points
        } 
    }
    catch (std::exception e){
        // No intersection, time to throw exception
    }

    return allPoints;
}

// Assigns values to the constructor
CurvePoint::CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
    this->x = x;
    this->y = y;
    this->moveSpeed = moveSpeed;
    this->turnSpeed = turnSpeed;
    this->followDistance = followDistance;
    this->slowDownTurnRadians = slowDownTurnRadians;
    this->slowDownTurnAmount = slowDownTurnAmount;

}

// Assigns values to the class
CurvePoint::CurvePoint(const CurvePoint &thisPoint){
    x = thisPoint.x;
    y = thisPoint.y;
    moveSpeed = thisPoint.moveSpeed;
    turnSpeed = thisPoint.turnSpeed;
    followDistance = thisPoint.followDistance;
    slowDownTurnRadians = thisPoint.slowDownTurnRadians;
    slowDownTurnAmount = thisPoint.slowDownTurnAmount;
}

// Sets a new point to the current x and y val
Point CurvePoint::toPoint(){
    Point newPoint;
    newPoint.setX(x);
    newPoint.setY(y);
    return newPoint;
}

// Sets x and y to be the current x and y val
void CurvePoint::setPoint(Point point)
{
  x = point.getX();
  y = point.getY();
}

// Get the current follow distance
double CurvePoint::getFollowDistance(){
  return followDistance;
}

// Get x
double CurvePoint::getX(){
  return x;
}
// Get y
double CurvePoint::getY(){
  return y;
}

CurvePoint getFollowPointPath(std::vector<CurvePoint> pathPoints, Point robotLocation, double followRadius){
    CurvePoint followMe(pathPoints.at(0));
    std::vector<Point> intersections;
    std::vector<Point> intersections2;

    for (int i = 0; i < pathPoints.size() - 1; i++)
    { 
        CurvePoint startLine = pathPoints.at(i);
        CurvePoint endLine = pathPoints.at(i + 1);

        intersections = LineCircleIntersection(robotLocation, pathPoints.at(i).getFollowDistance(), startLine.toPoint(), endLine.toPoint());

        if (intersections.size() == 1){
            followMe.setPoint(intersections.at(0));
        }
        else if (intersections.size() == 2){

            Point one = intersections.at(0);
            Point two = intersections.at(1);

            double distanceOne = sqrtf(pow((endLine.getX() - one.getX()), 2) + pow((endLine.getY() - one.getY()), 2));
            double distanceTwo = sqrtf(pow((endLine.getX() - two.getX()), 2) + pow((endLine.getY() - two.getY()), 2));

            if (distanceOne < distanceTwo){
                followMe.setPoint(one);
            }
            else{
                followMe.setPoint(two);
            }
        }
    }

  return followMe;
}

// Follows Pure Pursuit path
void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle){
    Point robotPosition;
    MotionAlgorithms CurveHandler;
    robotPosition.setX(gx);
    robotPosition.setY(gy);

    CurvePoint followMe = getFollowPointPath(allPoints, robotPosition, allPoints.at(0).getFollowDistance());
    CurveHandler.GTP_Movement(followMe.getX(), followMe.getY()); // Go to point
}



const int maxSpeed = 9000;

void ArcMovement::CurveToPoint(double targetX, double targetY){

    double targetTheta = atan2f(targetX - gx, targetY - gy);
    bool rightTurn = false;

    targetTheta = (targetTheta - ImuMon());
    targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;

    if (targetTheta >= 0){ 
        rightTurn = true;
    }
    else{ 
        rightTurn = false;
    }

    if (fabs(targetTheta) < 1.5){ 
        utility::leftvreq(maxSpeed);
        utility::rightvreq(maxSpeed);
    }
    else if (rightTurn){
        utility::leftvreq(maxSpeed);
        utility::rightvreq(maxSpeed * SpeedCompensator);
    }
    else{
        utility::leftvreq(maxSpeed * SpeedCompensator);
        utility::rightvreq(maxSpeed);
    }
      
  pros::delay(10);
}
