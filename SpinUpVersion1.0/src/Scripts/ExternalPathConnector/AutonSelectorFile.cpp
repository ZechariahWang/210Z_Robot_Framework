#include "main.h"

void Run_MTRP_Debug(){
    Debug_MTRP();
}

void Run_PID_Debug(){
    PID_Debug();
}

void StandardAuton(){
	MotionAlgorithms Auton_Framework;

	Auton_Framework.MTRP(0, 0, 90, 45);
	Auton_Framework.overRideCoordinatePos(0, 0);

	Auton_Framework.MTRP(65, 61, 90, -90);
	Auton_Framework.overRideCoordinatePos(0, 0);
}

void SkillsPath(){
    // Skills function will go here
}

void AutonSelectorPrimary(const int autonType){
    switch (autonType)
    {
    case 0: // doesnt run
        a_rightSideDisk();
        break;
    case 1:
		a_leftSideDisk();
        break;
    case 2:
		a_rightSideDisk();
        break;
    case 3:
		a_leftSideDisk();
        break;
    case 4:
		a_leftSideDisk();
        break;
    case 5:
		a_leftSideDisk();
        break;
    case 6:
		a_leftSideDisk();
        break;
    case 7:
		a_leftSideDisk();
        break;
    case 8:
		a_leftSideDisk();
        break;
    case 9:
		a_leftSideDisk();
        break;
    case 10:
		a_leftSideDisk();
        break;
    default:
		a_leftSideDisk();
        break;
    }
}