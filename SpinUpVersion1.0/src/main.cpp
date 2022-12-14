#include "main.h"
#include "pros/motors.h"
#include "vector"
#include "variant"
#include "array"

const unsigned long int time = 100000; // Time until initialize phase ends. Effectively infinite.
const unsigned short int delayAmount = 10; // Dont overload the CPU during OP control

// wheres my dad ඞ

//-- LVGL object pointer initialization //--
lv_obj_t *displayDataL1;
lv_obj_t *displayDataL2;
lv_obj_t *displayDataL3;
lv_obj_t *displayDataL4;
lv_obj_t *displayDataL5;
lv_obj_t *debugLine1;
lv_obj_t *debugLine2;
lv_obj_t *finalizeAutonButton;
lv_obj_t *prevAutonButton;
lv_obj_t *nextAutonButton;

lv_obj_t *infoDisplay;
lv_obj_t *infoPage = lv_page_create(lv_scr_act(), NULL);

//-- LVGL on input functions //--
static lv_res_t btn_rel_action(lv_obj_t *btn){
	static bool pressed = true;
	if (pressed) {
		AutonFinalized = 1;
	} 
	return 0;
}

//-- LVGL prev auton selector functions
static lv_res_t onPrevPress(lv_obj_t *btn){
    SelectedAuton -= 1;
    if (SelectedAuton >= 11){
        SelectedAuton = 1;
    }
    else if (SelectedAuton <= 0){
        SelectedAuton = 10;
	}
	return 1;
}

//-- LVGL next auton selector functions
static lv_res_t onNextPress(lv_obj_t *btn){
	SelectedAuton += 1;
    if (SelectedAuton >= 11){
        SelectedAuton = 1;
    }
    else if (SelectedAuton <= 0){
        SelectedAuton = 10;
    }
	return 1;
}

void initialize() { // Init function control

	//-- New style initiation //--
    static lv_style_t style_new;                         
    lv_style_copy(&style_new, &lv_style_pretty);         
    style_new.body.radius = LV_RADIUS_CIRCLE;            
    style_new.body.main_color = LV_COLOR_BLACK;         
    style_new.body.grad_color = LV_COLOR_GRAY;         
    style_new.body.shadow.width = 8;                   
    style_new.body.border.width = 2;                    
    style_new.text.color = LV_COLOR_WHITE;                 

	//-- Info page style initiation //--
    static lv_style_t style_infoPage;                         
    lv_style_copy(&style_infoPage, &lv_style_pretty);              
    style_infoPage.body.main_color = lv_color_hsv_to_rgb(0, 0, 7);   
    style_infoPage.body.grad_color = lv_color_hsv_to_rgb(0, 0, 7);    
    style_infoPage.body.border.width = 1;   
	style_infoPage.text.color = LV_COLOR_WHITE; 

	lv_obj_set_size(infoPage, 500, 300);
	lv_obj_align(infoPage, NULL, LV_ALIGN_CENTER, 0, 30);  
	lv_obj_set_style(infoPage, &style_infoPage);

	//-- Debug Line //--
	debugLine1 = lv_label_create(infoPage, NULL);
    lv_label_set_text(debugLine1, " ");
    lv_obj_align(debugLine1, NULL, LV_ALIGN_CENTER, -200, -10);

	//-- Data line 1 //--
	displayDataL1 = lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL1, " ");
    lv_obj_align(displayDataL1, NULL, LV_ALIGN_CENTER, -200, -20);

	//-- Data line 2 //--
	displayDataL2= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL2, " ");
    lv_obj_align(displayDataL2, NULL, LV_ALIGN_CENTER, -200, -30);

	//-- Data line 3 //--
	displayDataL3= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL3, " ");
    lv_obj_align(displayDataL3, NULL, LV_ALIGN_CENTER, -200, -40);

	//-- Data line 4 //--
	displayDataL4= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL4, " ");
    lv_obj_align(displayDataL4, NULL, LV_ALIGN_CENTER, -200, -50);

	//-- Data line 5 //--
	displayDataL5= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL5, " ");
    lv_obj_align(displayDataL5, NULL, LV_ALIGN_CENTER, -200, -60);

	//-- Select Auton button //--
	finalizeAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(finalizeAutonButton, LV_BTN_ACTION_CLICK, btn_rel_action); 
    lv_obj_align(finalizeAutonButton, NULL, LV_ALIGN_CENTER, -5, 100);
	lv_obj_set_height(finalizeAutonButton, 40);
	lv_obj_set_width(finalizeAutonButton, 150);

	lv_obj_t *buttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(buttonText, "");  /*Set the text*/
    lv_obj_set_x(buttonText, 50); 

    buttonText = lv_label_create(finalizeAutonButton, NULL);
    lv_label_set_text(buttonText, SYMBOL_UPLOAD " SELECT");

	//-- Prev Auton button //--
	prevAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(prevAutonButton, LV_BTN_ACTION_CLICK, onPrevPress); 
    lv_obj_align(prevAutonButton, NULL, LV_ALIGN_CENTER, -140, 100);
	lv_obj_set_height(prevAutonButton, 40);
	lv_obj_set_width(prevAutonButton, 120);

	lv_obj_t *prevbuttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(prevbuttonText, "");  /*Set the text*/
    lv_obj_set_x(prevbuttonText, 50); 

    prevbuttonText = lv_label_create(prevAutonButton, NULL);
    lv_label_set_text(prevbuttonText, SYMBOL_PREV " PREV");

	//-- Next Auton button //--
	nextAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(nextAutonButton, LV_BTN_ACTION_CLICK, onNextPress); 
    lv_obj_align(nextAutonButton, NULL, LV_ALIGN_CENTER, 160, 100);
	lv_obj_set_height(nextAutonButton, 40);
	lv_obj_set_width(nextAutonButton, 120);

	lv_obj_t *nextbuttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(nextbuttonText, "");  /*Set the text*/
    lv_obj_set_x(nextbuttonText, 50); 

    nextbuttonText = lv_label_create(nextAutonButton, NULL);
    lv_label_set_text(nextbuttonText, SYMBOL_NEXT " NEXT");
 
	//-- Reset sensors and auton selector init //--
	pros::delay(3000);
	FinalizeAuton Init_Process;
	Init_Process.ResetAllPrimarySensors();
    Expansion.set_value(true);
	Launcher.set_value(true);
	OuterShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	InnerShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	//Init_Process.ReceiveInput(time); // 10000 = 10 seconds
}

//--DONT TOUCH THESE FUNCTIONS--\*
void disabled() {}
void competition_initialize() {}
//------------------------------\*

// 500 1500

double counter_yes = 0;
void shoot(double power){
	for (int i = 0; i < 2; i++){
		OuterShooter.move_voltage(power);
    	InnerShooter.move_voltage(power);
		Launcher.set_value(false);
		pros::delay(500);
		Launcher.set_value(true);
		if (counter_yes == 0){
			OuterShooter.move_voltage(power);
			InnerShooter.move_voltage(power);
		}
		else{
			OuterShooter.move_voltage(power + 100);
			InnerShooter.move_voltage(power + 100);
		}
		pros::delay(2500);
		counter_yes++;
	}
}

void leftSide(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	eclipse_PID PID_eclipse; // PID class
	PID pid;

	pros::delay(100);

	DiskIntakeTop.move_voltage(12000);
    DiskIntakeBot.move_voltage(12000);

    OuterShooter.move_voltage(9100);
    InnerShooter.move_voltage(9100);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(-3, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(2, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 35);
	PID_eclipse.combined_TurnPID(-11.5, 60);

	pros::delay(2400);

	shoot(9100);

	// PID_eclipse.set_turn_pid_targets(3, 0.003, 35);
	// PID_eclipse.combined_TurnPID(45, 60);

    // OuterShooter.move_voltage(0);
    // InnerShooter.move_voltage(0);

	// PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	// PID_eclipse.combined_TranslationPID(140, 90, -90, true, false);
	// pros::delay(100);

	// PID_eclipse.set_turn_pid_targets(5, 0.003, 60);
	// PID_eclipse.combined_TurnPID(-90, 60);

	// PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	// PID_eclipse.combined_TranslationPID(-4, 90, -90, true, false);
	// pros::delay(100);

	// PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	// PID_eclipse.combined_TranslationPID(2, 90, -90, true, false);
	// pros::delay(100);
}

void rightSide(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	eclipse_PID PID_eclipse; // PID class
	PID pid;
	pros::delay(100);

	DiskIntakeTop.move_voltage(5000);
    DiskIntakeBot.move_voltage(5000);

    OuterShooter.move_voltage(8700);
    InnerShooter.move_voltage(8700);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(3, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 35);
	PID_eclipse.combined_TurnPID(-90, 60);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(-30, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 35);
	PID_eclipse.combined_TurnPID(0, 60);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(-6, 90, -90, true, false);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(3, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 35);
	PID_eclipse.combined_TurnPID(5.5, 60);

	pros::delay(2500);

	shoot(8700);
}

void awp(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	eclipse_PID PID_eclipse; // PID class
	PID pid;

	DiskIntakeTop.move_voltage(12000);
    DiskIntakeBot.move_voltage(12000);

    OuterShooter.move_voltage(9000);
    InnerShooter.move_voltage(9000);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(-3, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(2, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 35);
	PID_eclipse.combined_TurnPID(-10.5, 60);

	shoot(9000);

	PID_eclipse.set_turn_pid_targets(3, 0.003, 35);
	PID_eclipse.combined_TurnPID(45, 60);

    OuterShooter.move_voltage(0);
    InnerShooter.move_voltage(0);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(140, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_turn_pid_targets(5, 0.003, 60);
	PID_eclipse.combined_TurnPID(-90, 60);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(-4, 90, -90, true, false);
	pros::delay(100);

	PID_eclipse.set_translation_pid_targets(0.45, 0, 5, 1.5);
	PID_eclipse.combined_TranslationPID(2, 90, -90, true, false);
	pros::delay(100);
}

// PID 1 inch = 34.4
// PID Units: inches

void autonomous(){  // Autonomous function control
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	eclipse_PID PID_eclipse; // PID class
	PID pid;
	SecondOdometry();
	DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Auton_Framework.overRideCoordinatePos(0, 0);
	PID_eclipse.set_constants(3, 2.3, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);

	//Init_Process.SelectAuton(); // For Auton Selector

	//awp();

	//leftSide();

	rightSide();

}

void opcontrol(){ // Driver control function
	Op_EndGame Op_Framework; // OP control framework class
	MotionAlgorithms Auton_Framework; // Auton framework class
	Init_AutonSwitchMain Init; // Init class framework
	FinalizeAuton data; // Data class
	char buffer[300];
	while (true){
		Op_Framework.HDriveControl(); // Drivetrain control
		Op_Framework.PowerIntake(); // Intake control
		Op_Framework.LaunchDisk(); // Disk control
		Op_Framework.SetPowerAmount(); // Power control
		Op_Framework.PowerShooter(); // Shooter control OVERRIDE 
		Op_Framework.setMotorType();
		Op_Framework.InitiateExpansion();
		//Op_Framework.TBH_AlgorithmControl(); // Shooter control TBH ALGORITHM
		//Op_Framework.p_flywheel();

		double ms = pros::millis();
		// char buffer[300];
		// sprintf(buffer, "ms since startup: %f", ms);
		// lv_label_set_text(debugLine1, buffer);
		data.DisplayData(); // Display robot stats and info
		pros::Task OdomTask(SecondOdometry); // Multitasking for odom
		pros::delay(delayAmount); // Dont hog CPU ;)
	}
}

//lol ඞ
//background as black as kartik