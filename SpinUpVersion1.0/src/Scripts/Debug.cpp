#include "main.h"


void PID_Debug(){
    MotionAlgorithms PID;
    PID.TranslationPID(2000, 12000);
    PID.TurnPID(45);
    PID.TurnPID(-45);
    PID.TurnPID(0);
    PID.TranslationPID(-2000, 12000);
}

void Debug_GTC(){
    GoToCoordPos(-20, -20, 90, 480, 480, 2, 60);
	GoToCoordPos(0, 0, 0, 480, 480, 2, 60);
	GoToCoordPos(15, 20, -90, 480, 480, 2, 60);
    GoToCoordPos(0, 0, 0, 480, 480, 2, 60);
    GoToCoordPos(35, 10, 0, 480, 480, 2, 60);
}

void Debug_MTRP(){
    MotionAlgorithms Auton_Framework;
	Auton_Framework.MTRP(5, 15, 90, 180);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(-30, -10, 90, 90);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(20, 10, 90, 0);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(10, -10, 90, 0);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(20, 20, 90, 0);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(-20, -20, 90, -180);
	Auton_Framework.overRideCoordinatePos(0, 0);
}