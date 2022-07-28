#include "main.h"

namespace utility
{
    void sgn(double num);
    void stop();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void fullreset(double resetval, bool imu);
}

void ForwardPID(int target);
void TurnPID(double t_theta);