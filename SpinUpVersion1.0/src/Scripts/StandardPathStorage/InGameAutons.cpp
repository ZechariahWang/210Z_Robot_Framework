#include "main.h"

void shoot_y(){
	Launcher.set_value(true);
	pros::delay(500);
	Launcher.set_value(false);
	pros::delay(500);
}

void a_rightSideDisk(){ // one during wp
    MotionAlgorithms Auton_Framework; // Auton framework class
    FinalizeAuton Init_Process; // Init framework class
    eclipse_PID PID_eclipse; // PID class
    // stuff

	DiskIntake.move_voltage(9000);
    OuterShooter.move_voltage(11300);

	pros::delay(1500);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(-3, 400, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(4, 200, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(2.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(5, 12000);
	pros::delay(500);

	shoot_y();

	PID_eclipse.set_turn_pid_targets(2.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(-58, 12000);
	pros::delay(500);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 2.3);
	PID_eclipse.combined_TranslationPID(32, 170, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(3.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(-38, 12000);
	pros::delay(100);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(24, 200, -200, true, false);
	pros::delay(100);

    OuterShooter.move_voltage(11500);

	PID_eclipse.set_turn_pid_targets(3, 0, 2.4);
	PID_eclipse.combined_TurnPID(45, 12000);

	shoot_y();
}

void a_leftSideDisk(){ // one after wp
    MotionAlgorithms Auton_Framework; // Auton framework class
    FinalizeAuton Init_Process; // Init framework class
    eclipse_PID PID_eclipse; // PID class
    // stuff

	DiskIntake.move_voltage(7000);
    OuterShooter.move_voltage(11000);

	pros::delay(1700);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(-3, 400, -200, true, false);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(4, 200, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(2.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(-12, 12000);
	pros::delay(500);

	shoot_y();
}