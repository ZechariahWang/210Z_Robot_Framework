#include "main.h"

class AngleWrap_C{
    private:
        bool init;
    public:
        int AngleWrap(double angle);
};

class ArcMovement{
    private:
        bool init;
    public:
        void _CurveToPoint(double targetX, double targetY);
};

class Point{
    double xCoord;
    double yCoord;
    public:
        void setX(double x) {xCoord = x;}
        void setY(double y) {yCoord = y;}
        double getX(){
            return xCoord;
        }
        double getY(){
            return yCoord;
        }
};

class CurvePoint{
    private:
        bool init;
    public:

        double x;
        double y;
        double moveSpeed;
        double turnSpeed;
        double followDistance;
        double slowDownTurnRadians;
        double slowDownTurnAmount;
        
        CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount);
        CurvePoint(const CurvePoint &thisPoint);

        Point toPoint();
        void setPoint(Point point);
        double getFollowDistance();
        double getX();
        double getY();

};

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle);