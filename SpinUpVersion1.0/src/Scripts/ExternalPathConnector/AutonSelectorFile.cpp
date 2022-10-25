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
    case 0:
        Run_PID_Debug();
        break;
    case 1:
		Run_MTRP_Debug();
        break;
    case 2:
		StandardAuton();
        break;
    case 3:
		StandardAuton();
        break;
    case 4:
		StandardAuton();
        break;
    case 5:
		StandardAuton();
        break;
    case 6:
		StandardAuton();
        break;
    case 7:
		StandardAuton();
        break;
    case 8:
		StandardAuton();
        break;
    case 9:
		StandardAuton();
        break;
    case 10:
		SkillsPath();
        break;
    default:
		SkillsPath();
        break;
    }
}