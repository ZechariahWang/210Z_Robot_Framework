#include "main.h"

int AngleWrap_C::AngleWrap(double angle){
    while (angle < M_PI){
        angle += 2 * M_PI;
    }
    while (angle > M_PI){
        angle -= 2 * M_PI;
    }

    return angle;
}

std::vector<Point> LineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
    if (abs(linePoint1.getY() - linePoint2.getY()) < 0.003){
        linePoint1.setY(linePoint2.getY() + 0.003);
    }
    if (abs(linePoint1.getX() - linePoint2.getX()) < 0.003){
        linePoint1.setX(linePoint2.getX() + 0.003);
    }

    double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());
    double quadraticA = 1.0 + pow(m1, 2);

    double x1 = linePoint1.getX() - circleCenter.getX();
    double y1 = linePoint1.getY() - circleCenter.getY();

    double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);
    double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

    std::vector<Point> allPoints;

    try
    {
        double xRoot1 = (-quadraticB + sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot1 = m1 * (xRoot1 - x1) + y1;

        xRoot1 += circleCenter.getX();
        yRoot1 += circleCenter.getY();

        double minX = linePoint1.getX() < linePoint2.getX() ? linePoint1.getX() : linePoint2.getX();
        double maxX = linePoint1.getY() < linePoint2.getY() ? linePoint1.getY() : linePoint2.getY();

        if (xRoot1 > minX && xRoot1 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot1);
            newPoint.setY(yRoot1);
            allPoints.push_back(newPoint);
        }
        else{
            // No Points
        }

        double xRoot2 = (-quadraticB - sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot2 = m1 * (xRoot2 - x1) + y1;

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

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle){
    Point robotPosition;
    robotPosition.setX(gx);
    robotPosition.setY(gy);

    CurvePoint followMe = getFollowPointPath(allPoints, robotPosition, allPoints.at(0).getFollowDistance());
    // Move to point function with the following params: followMe.getX(), followMe.getY(), extra params here.
}
