#include "main.h"

void Run_MTRP_Debug(){
    Debug_MTRP();
}

void Run_PID_Debug(){
    PID_Debug();
}

void StandardAuton(){
	MotionAlgorithms Auton_Framework;
	Auton_Framework.TurnPID(45);
}

void SkillsPath(){
    // Skills function will go here
}

void AutonSelectorPrimary(const int autonType){
    switch (autonType)
    {
    case 0:
		default_A();
        break;
    case 1:
		Run_MTRP_Debug();
        break;
    case 2:
		Run_PID_Debug();
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
		StandardAuton();
        break;
    default:
		SkillsPath();
        break;
    }
}