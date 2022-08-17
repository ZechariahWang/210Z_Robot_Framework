#include "main.h"

// im kinda lost
void DebugStrafe(){
	const int delayAmount = 1000;
	const int velocity = 127;
	DriveFrontLeft.move_velocity(velocity);
	pros::delay(delayAmount);
	DriveFrontRight.move_velocity(-velocity);
	pros::delay(delayAmount);
	DriveBackLeft.move_velocity(-velocity);
	pros::delay(delayAmount);
	DriveBackRight.move_velocity(velocity);
}

void PID_Debug(){
	TurnPID(90);
    pros::delay(500);
	TurnPID(0);
	// ForwardPID(2000);
	// ForwardPID(-2000);
    // ForwardPID(2000);
	// ForwardPID(-2000);
}

void DebugGTC(){
    GoToCoordPos(-20, -20, 90, 480, 480, 2, 60);
	GoToCoordPos(0, 0, 0, 480, 480, 2, 60);
	GoToCoordPos(15, 20, -90, 480, 480, 2, 60);
    GoToCoordPos(0, 0, 0, 480, 480, 2, 60);
    GoToCoordPos(35, 10, 0, 480, 480, 2, 60);
}