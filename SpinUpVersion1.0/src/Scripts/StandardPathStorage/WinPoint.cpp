#include "main.h"

void SoloWinPoint(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	eclipse_PID PID_eclipse; // PID class

	DiskIntake.move_voltage(9000);
    OuterShooter.move_voltage(11000);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(-3, 400, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 1.2);
	PID_eclipse.combined_TranslationPID(4, 200, -200, true, false);
	pros::delay(100);

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

	PID_eclipse.set_turn_pid_targets(3, 0, 2.4);
	PID_eclipse.combined_TurnPID(50, 12000);

	for (int i = 0; i < 1; i++){
		Launcher.set_value(true);
		pros::delay(500);
		Launcher.set_value(false);
		pros::delay(500);
	}
	// // shoot lol

	PID_eclipse.set_turn_pid_targets(3, 0, 2.4);
	PID_eclipse.combined_TurnPID(-55, 12000);

	DiskIntake.move_voltage(10000);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 2.3);
	PID_eclipse.combined_TranslationPID(75, 300, -200, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(2.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(0, 12000);

	PID_eclipse.set_turn_pid_targets(2.6, 0, 2.4);
	PID_eclipse.combined_TurnPID(90, 12000);

    OuterShooter.move_voltage(0);

	PID_eclipse.set_pid_targets(1, 0, 1.2, 2.3);
	PID_eclipse.combined_TranslationPID(-20, 300, -200, true, false);
	pros::delay(1000);

	DiskIntake.move_voltage(0);

}
