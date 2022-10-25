#include "main.h"

MotionAlgorithms Auton_Framework; // Auton framework class
FinalizeAuton Init_Process; // Init framework class
eclipse_PID PID_eclipse; // PID class

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
	// Auton_Framework.MTRP(5, 15, 90, 180);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.MTRP(-30, -10, 90, 90);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.MTRP(20, 10, 90, 0);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.MTRP(10, -10, 90, 0);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.MTRP(20, 20, 90, 0);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.MTRP(-20, -20, 90, -180);
	// Auton_Framework.overRideCoordinatePos(0, 0);
}

void debug_eclipsePID(){
	// PID_eclipse.set_pid_targets(1, 0.1, 1.2, 4);
	// PID_eclipse.combined_TranslationPID(3000, 500, true);

	// Auton_Framework.TurnPID(0);

	// PID_eclipse.set_pid_targets(1, 0.1, 0.4, 4);
	// PID_eclipse.combined_TranslationPID(-3000, 500, true);

	// Auton_Framework.TurnPID(90);

	// Auton_Framework.TurnPID(-90);

	// Auton_Framework.TurnPID(0);

	// PID_eclipse.set_pid_targets(1, 0.1, 1.2, 4);
	// PID_eclipse.combined_TranslationPID(2000, 500, true);

	// Auton_Framework.overRideCoordinatePos(0, 0);
	// Auton_Framework.MTRP(-20, 20, 90, 45);
	// Auton_Framework.overRideCoordinatePos(0, 0);

	// Auton_Framework.TurnPID(0);

	// Auton_Framework.overRideCoordinatePos(0, 0);
	// Auton_Framework.MTRP(20, -30, 180, 45);
	// Auton_Framework.overRideCoordinatePos(0, 0);	
}