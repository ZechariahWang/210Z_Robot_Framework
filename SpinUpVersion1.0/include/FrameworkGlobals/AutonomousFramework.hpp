#include "main.h"
// Classes for the main autonomous functions, as well as some algorithms. Designated algorithms are found in the algorithm framework

class Auton_Init{
    private:
        bool init;
    public:
        void Initialize_AutonCommands();
        void overRideCoordinatePos(double new_gx, double new_gy);
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
        void MTRP(double tx, double ty, double targetHeading, double GlobalHeading);
        void MTRP_Movement(double tx, double ty, double targetHeading, double GlobalHeading);
        void NHMTP(double target_X, double target_Y);
        void GTP_Movement(double target_X, double target_Y);
};

class eclipse_PID{
    private:
        bool init;
    public:
        short int e_rkp = 0; // Heading correction kp
        short int e_kp = 0;
        short int e_ki = 0;
        short int e_kd = 0;
        short int e_current = 0;
        short int e_error = 0;
        short int e_prevError = 0;
        short int e_integral = 0;
        short int e_derivative = 0;
        short int e_timer = 0;

        short int e_target = 0;
        short int e_maxSpeed = 0;

        bool e_headingStat = false;

        void reset_pid_targets();
        void reset_pid_inputs();
        void set_constants(double n_wheelDiameter, double n_gearRatio, double n_motorCartridge);
        void set_pid_targets(double kp, double ki, double kd, double rkp);
        int find_min_angle(int targetHeading, int currentrobotHeading);
        double compute_translation(double current);
        double translation_pid_task(int targetHeading, bool headingEnabled);
        void eclipse_TranslationPID(short int target, short int maxSpeed, bool headingStat);
        void combined_TranslationPID(short int target, short int maxSpeed, short int minSpeed, bool headingStat, bool averagePosStat);
        void combined_TurnPID(double te_theta, double turnSpeed);
        void reset_combined_targets();
        void reset_turn_combined_targets();
        void set_turn_pid_targets(double kp, double ki, double kd);
};

namespace auton_utility
{
    int sgn(double num);
    void stop();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void fullreset(double resetval, bool imu);
}

namespace utility
{
    int sgn(double num);
    void stop();
    void stop_v();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void leftvelreq(double velocity);
    void rightvelreq(double velocity);
    void leftvoltagereq(double voltage);
    void rightvoltagereq(double voltage);
    void fullreset(double resetval, bool imu);
    void eclipse_fullreset(double resetval, bool imu);
}
