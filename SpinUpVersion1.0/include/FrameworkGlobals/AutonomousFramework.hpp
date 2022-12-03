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
        double te_kp                                   = 2;
        double te_ki                                   = 0.002;
        double te_kd                                   = 0;

        double te_derivative                           = 0;
        double te_integral                             = 0;
        double te_tolerance                            = 12;
        double te_error                                = 0;
        double te_previouserror                        = 0;
        double te_multiplier                           = 3000;
        double te_averageposition                      = 0;
        double te_averageHeading                       = 0;
        double te_FailSafeCounter                      = 0;
        int te_threshholdcounter                       = 0;

        // Combined Translation PID Settings
        double p_target = 0;
        double p_maxSpeed = 0;
        double p_headingStat =0;
        double p_kp                                    = 0.3; // 0.4
        double p_ki                                    = 0;
        double p_kd                                    = 0.03;
        double p_rkp                                   = 4;

        double p_derivative                            = 0;
        double p_integral                              = 0;
        double pt_tolerance                            = 90;
        double p_error                                 = 0;
        double p_previouserror                         = 0;
        double p_multiplier                            = 200;
        double p_averageposition                       = 0;
        double p_currentposition                       = 0;
        double p_averageHeading                        = 0;
        double p_FailSafeCounter                       = 0;
        int p_threshholdcounter                        = 0;

        double wheelDiameter                           = 0;
        double ratio                                   = 0;
        double cartridge                               = 0;
        double circumfrance                            = 0;
        double ticks_per_rev                           = 0;
        double ticks_per_inches                        = 0;

        eclipse_PID();
        void reset_pid_targets();
        void reset_pid_inputs();
        void set_constants(double n_wheelDiameter, double n_gearRatio, double n_motorCartridge);
        void set_translation_pid_targets(double kp, double ki, double kd, double rkp);
        int find_min_angle(int targetHeading, int currentrobotHeading);
        double compute_translation(double current);
        double translation_pid_task(int targetHeading, bool headingEnabled);
        void eclipse_TranslationPID(short int target, short int maxSpeed, bool headingStat);
        void combined_TranslationPID(short int target, short int maxSpeed, short int minSpeed, bool headingStat, bool averagePosStat);
        void combined_TurnPID(double te_theta, double turnSpeed);
        void reset_translation_combined_targets();
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
