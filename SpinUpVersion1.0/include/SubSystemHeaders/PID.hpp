#include "main.h"

void TranslationPID(int target, int maxVoltage);
float Turn_PID(double GTC_theta);
void TurnPID(double t_theta);
void ArcPID(double targetX, double targetY);

namespace utility
{
    int sgn(double num);
    void stop();
    void stop_v();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void leftvelreq(double velocity);
    void rightvelreq(double velocity);
    void fullreset(double resetval, bool imu);
}