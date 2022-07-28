#include "main.h"

extern double px;
extern double py;
extern double gx;
extern double gy;
extern double global_x;
extern double global_y;
extern double global_theta;

void PrimaryOdometry();
void StandardOdom();
void SecondOdometry();
double ImuMon();