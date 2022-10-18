#include "main.h"

extern double px;
extern double py;
extern double gx;
extern double gy;
extern double global_x;
extern double global_y;
extern double global_theta;
extern double d_deltaX;
extern double d_deltaY;
extern double globalTheta;

void PrimaryOdometry();
void StandardOdom();
void SecondOdometry();
double ImuMon();