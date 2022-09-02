#include "main.h"

class Auton_Init{
    private:
        bool init;
    public:
        void Initialize_AutonCommands();
};

class PID : public Auton_Init{
    private:
        bool init_PID;
    public:
        void TranslationPID(int target, int maxVoltage);
        float Turn_PID(double GTC_theta);
        void TurnPID(double t_theta);
        void ArcPID(double targetX, double targetY);

};

class Odometry : public PID{
    private:
        bool init_Odom;
    public:
        void StandardOdom();
        void SecondOdometryOLD();

};

class MotionAlgorithms : public Odometry{
    private:
        bool init_MotionAlg;
    public:
        void PurePursuitRunner(std::vector<std::array<double, 2>> Path);
        void TurnToPoint(int targetX, int targetY);
        void GoToCoordPos(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate);
        void MTRP(double tx, double ty);
};

namespace auton_utility
{
    int sgn(double num);
    void stop();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void fullreset(double resetval, bool imu);
}
