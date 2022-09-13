#include "main.h"

void TurnToPoint(int targetX, int targetY);
void PurePursuit(std::vector<std::array<double, 3>> path);
int SecondPurePursuit(std::vector<std::array<double, 2>> Path);
void PurePursuitRunner(std::vector<std::array<double, 2>> Path);
void GoToCoordPos(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate);
//void MTRP(double tx, double ty, double targetHeading);