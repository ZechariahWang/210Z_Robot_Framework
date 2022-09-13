#include "main.h"

void PurePursuitTestPath(){
    MotionAlgorithms curveHandler;
    std::vector<CurvePoint> Path;

    const double finalX = 24.3;
    const double finalY = 12;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(4.6, 7, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(18.6, 8.7, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(24, 10, 0, 0, 10, 5, 1);

    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    Path.push_back(newPoint1); 
    Path.push_back(newPoint2); 
    Path.push_back(newPoint3);
    Path.push_back(EndPos);

    while (true)
    {
      if (sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2)) < 11)
      {
        curveHandler.GTP_Movement(finalX, finalY);
        break;
      }
      FollowCurve(Path, 0);
    }
}