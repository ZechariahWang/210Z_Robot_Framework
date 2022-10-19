#include "main.h"

// Test path for Pure Pursuit Algorithm
void PurePursuitTestPath(){
    MotionAlgorithms curveHandler;
    std::vector<CurvePoint> Path;

    const double finalX = 60;
    const double finalY = 20;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);

    CurvePoint newPoint1(20, 20, 0, 0, 10, 5, 1);
    curveHandler.overRideCoordinatePos(20, 20);

    CurvePoint newPoint2(40, 20, 0, 0, 10, 5, 1);
    curveHandler.overRideCoordinatePos(20, 20);

    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    Path.push_back(newPoint1); 
    Path.push_back(EndPos);

    while (true)
    {
      SecondOdometry();
      if (sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2)) < 11)
      {
        curveHandler.MTRP(finalX, finalY, 0, 0);
        break;
      }
      FollowCurve(Path, 0);
    }
}