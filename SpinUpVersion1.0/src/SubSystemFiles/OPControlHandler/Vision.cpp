#include "main.h"

// vision control concept. No actual implementation yet
void vision_main_red(){
    const int maxlimit      = 150;
    const int minlimit      = -150;
    const int lockrange     = 80;
    const int defx          = 33; // || 37;
    const int defy          = 31488; // || 18688;
    const int redSIGint     = 1;

    int prevX               = 0;
    int prevY               = 0;
    
    pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(redSIGint, 7141, 8465, 7803, -1221, -669, -945, 3.000, 0);

    pros::vision_object_s_t rtn = vision_sensor.get_by_sig(0, redSIGint);
    int xcoord = rtn.x_middle_coord;
    int ycoord = rtn.y_middle_coord; 

    xcoord = 0;
    ycoord = 0;

    if ((xcoord == defx) && (ycoord == defy)) {
        // pros::lcd::print(3, "Past limit, idle");
    }
    else if((xcoord < lockrange) && (xcoord > -lockrange)){
        // pros::lcd::print(3, "Decision Overrided: Locked on");    
    }
    else if ((xcoord > 0) && (xcoord > lockrange) && (xcoord != defx)) {
        // pros::lcd::print(3, "Action: Move turret right");
    }
    else if((xcoord < 0) && (xcoord < -lockrange) && (xcoord != defx)) {
        // (3, "Action: Move turret left");
    }
    else{
        // pros::lcd::print(3, "Action: Spin Idle");
    }

    // pros::lcd::print(1, "red x: %d ", xcoord);
    // pros::lcd::print(2, "red y: %d ", ycoord);

}