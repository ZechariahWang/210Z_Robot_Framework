#include "main.h"

// Test path for Pure Pursuit Algorithm
void PurePursuitTestPath(){
    MotionAlgorithms curveHandler;
    FinalizeAuton data;
    std::vector<CurvePoint> Path;

    const double finalX = 40;
    const double finalY = 20;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);
    // CurvePoint newPoint1(40, 15, 0, 0, 10, 5, 1);
    // CurvePoint newPoint2(40, 30, 0, 0, 10, 5, 1);
    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    // Path.push_back(newPoint1); 
    // Path.push_back(newPoint2);
    Path.push_back(EndPos);

    while (true){
      SecondOdometry();
      data.DisplayData();
      if (sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2)) <= 20){
        curveHandler.MTRP(finalX, finalY, 0, 0);
        break;
      }
      FollowCurve(Path, 0);
    }
}