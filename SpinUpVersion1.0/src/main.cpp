#include "main.h"
#include "vector"
#include "variant"
#include "array"

// Finalize auton choice
void on_center_button() {
}

static lv_res_t btn_rel_action(lv_obj_t *btn){
	static bool pressed = true;
	if (pressed) {
		AutonFinalized = 1;
	} 
	return 0;
}

lv_obj_t *displayDataL1;
lv_obj_t *displayDataL2;
lv_obj_t *displayDataL3;
lv_obj_t *displayDataL4;
lv_obj_t *displayDataL5;

lv_obj_t *finalizeAutonButton;

lv_obj_t *infoDisplay;

lv_obj_t *infoPage = lv_page_create(lv_scr_act(), NULL);

void hi(){
	return;
}

void initialize() {
	// pros::lcd::set_text(1, "running init!");
	// pros::lcd::initialize();
	// pros::lcd::register_btn1_cb(on_center_button);
	short int time = 5000;  

    static lv_style_t style_new;                         /*Styles can't be local variables*/
    lv_style_copy(&style_new, &lv_style_pretty);         /*Copy a built-in style as a starting point*/
    style_new.body.radius = LV_RADIUS_CIRCLE;            /*Fully round corners*/
    style_new.body.main_color = LV_COLOR_BLACK;          /*White main color*/
    style_new.body.grad_color = LV_COLOR_GRAY;           /*Blue gradient color*/
    style_new.body.shadow.width = 8;                     /*8 px shadow*/
    style_new.body.border.width = 2;                     /*2 px border width*/
    style_new.text.color = LV_COLOR_WHITE;                 /*Red text color */

    static lv_style_t style_infoPage;                         /*Styles can't be local variables*/
    lv_style_copy(&style_infoPage, &lv_style_pretty);              
    style_infoPage.body.main_color = lv_color_hsv_to_rgb(0, 0, 7);   
    style_infoPage.body.grad_color = lv_color_hsv_to_rgb(0, 0, 7);    
    style_infoPage.body.border.width = 1;   
	style_infoPage.text.color = LV_COLOR_WHITE; 

	lv_obj_set_size(infoPage, 500, 300);
	lv_obj_align(infoPage, NULL, LV_ALIGN_CENTER, 0, 30);  
	lv_obj_set_style(infoPage, &style_infoPage);

	displayDataL1 = lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL1, " ");
    lv_obj_align(displayDataL1, NULL, LV_ALIGN_CENTER, -200, -20);

	displayDataL2= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL2, " ");
    lv_obj_align(displayDataL2, NULL, LV_ALIGN_CENTER, -200, -20);

	displayDataL3= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL3, " ");
    lv_obj_align(displayDataL3, NULL, LV_ALIGN_CENTER, -200, -30);

	displayDataL4= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL4, " ");
    lv_obj_align(displayDataL4, NULL, LV_ALIGN_CENTER, -200, -40);

	displayDataL5= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL5, " ");
    lv_obj_align(displayDataL5, NULL, LV_ALIGN_CENTER, -200, -50);

	finalizeAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(finalizeAutonButton, LV_BTN_ACTION_CLICK, btn_rel_action); 
    lv_obj_align(finalizeAutonButton, NULL, LV_ALIGN_CENTER, 0, 100);
	lv_obj_set_height(finalizeAutonButton, 40);
	lv_obj_set_width(finalizeAutonButton, 150);

	lv_obj_t *buttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(buttonText, "");  /*Set the text*/
    lv_obj_set_x(buttonText, 50); 

    buttonText = lv_label_create(finalizeAutonButton, NULL);
    lv_label_set_text(buttonText, SYMBOL_UPLOAD " SELECT");

	FinalizeAuton Init_Process;
	Init_Process.ResetAllPrimarySensors();
	Init_Process.ReceiveInput(100000); // 10000 = 10 seconds
}

//--DONT TOUCH THESE FUNCTIONS--\*
void disabled() {}
void competition_initialize() {}
//------------------------------\*

void autonomous(){
	MotionAlgorithms Auton_Framework;
	FinalizeAuton Init_Process;
	eclipse_PID PID;
	SecondOdometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	imu_sensor.set_rotation(0);
	//Init_Process.SelectAuton(); // For Auton Selector

	PID.set_pid_targets(0.03, 0, 0.03, 4);
	PID.reset_pid_targets();
	PID.reset_pid_inputs();
	PID.eclipse_TranslationPID(5000, 127, true);


	

}

const unsigned short int delayAmount = 10;
void opcontrol(){

	Op_SetPowerAmount Op_Framework;
	MotionAlgorithms Auton_Framework;
	Init_AutonSwitchMain Init;
	FinalizeAuton data;

	while (true){
		Op_Framework.HDriveControl(); // Drivetrain control
		Op_Framework.PowerShooter(); // Shooter control
		Op_Framework.PowerIntake(); // Intake control
		Op_Framework.LaunchDisk(); // Disk control
		Op_Framework.SetPowerAmount(); // Power control

		data.DisplayData();
		pros::Task OdomTask(SecondOdometry);
		pros::delay(delayAmount);
	}
}
