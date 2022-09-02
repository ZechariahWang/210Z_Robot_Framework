#include "main.h"
#include "vector"
#include "variant"
#include "array"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(3, "I was pressed!");
	} else {
		pros::lcd::clear_line(3);
	}
}

void otherstuff(){
	std::vector<std::array<double, 2>> Path = {{0, 0}, {0, 100}, {100, 100}};
	std::vector<std::array<double, 3>> Pathsecond = {{0, 0, 0}, {0, 20, 90}};
	std::array<double, 2> GoalPoint;
	// GoToCoordPos(5, 10, 90, 480, 480);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::set_text(1, "running init!");
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
	short int time = 5000;

	FinalizeAuton Init_Process;

	Init_Process.ResetAllPrimarySensors();
	Init_Process.ReceiveInput(5000); // 10000 = 10 seconds
	Init_Process.SelectAuton();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
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

void autonomous(){
	MotionAlgorithms Auton_Framework;
	pros::lcd::print(6, "Auton global: %d", globalAuton);

    // switch (globalAuton)
    // {
    // case 0:
	// 	default_A();
    //     break;
    // case 1:
	// 	A_1();
    //     break;
    // case 2:
	// 	A_2();
    //     break;
    // case 3:
	// 	A_3();
    //     break;
    // case 4:
	// 	default_A();
    //     break;
    // case 5:
	// 	default_A();
    //     break;
    // case 6:
	// 	default_A();
    //     break;
    // case 7:
	// 	default_A();
    //     break;
    // case 8:
	// 	default_A();
    //     break;
    // case 9:
	// 	default_A();
    //     break;
    // case 10:
	// 	default_A();
    //     break;
    // default:
	// 	default_A();
    //     break;
    // }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

const unsigned short int delayAmount = 10;

void opcontrol(){

	Op_SetPowerAmount Op_Framework;
	MotionAlgorithms Auton_Framework;
	Init_AutonSwitchMain Init;

	while (true){
		Op_Framework.HDriveControl();
		Op_Framework.PowerShooter();
		Op_Framework.PowerIntake();
		Op_Framework.LaunchDisk();
		Op_Framework.SetPowerAmount();

		// Init.ReceiveInput();
		pros::Task OdomTask(SecondOdometry);
		pros::delay(delayAmount);
	}
}
