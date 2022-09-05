#include "main.h"

void default_A(){
	MotionAlgorithms Auton_Framework;
	Auton_Framework.TranslationPID(2000, 12000);
}

void A_1(){
	MotionAlgorithms Auton_Framework;
	Auton_Framework.TurnPID(45);
}

void A_2(){
	MotionAlgorithms Auton_Framework;
	Auton_Framework.TurnPID(90);
}

void A_3(){
	MotionAlgorithms Auton_Framework;
	Auton_Framework.TurnPID(180);
}

void AutonSelectorPrimary(const int autonType){
    switch (autonType)
    {
    case 0:
		default_A();
        break;
    case 1:
		A_1();
        break;
    case 2:
		A_2();
        break;
    case 3:
		A_3();
        break;
    case 4:
		default_A();
        break;
    case 5:
		default_A();
        break;
    case 6:
		default_A();
        break;
    case 7:
		default_A();
        break;
    case 8:
		default_A();
        break;
    case 9:
		default_A();
        break;
    case 10:
		default_A();
        break;
    default:
		default_A();
        break;
    }
}