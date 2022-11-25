#include "main.h"

class Op_DTControl{
    private:
        int init;
    public:
        void HDriveControl();
        void XDriveTrainControl();
        void MecanumDriveControl();
};

class Op_PowerShooter : public Op_DTControl{
    private:
        int init;
    public:
        void PowerShooter();
        void TBH_AlgorithmControl();
};

class Op_PowerIntake : public Op_PowerShooter{
    private:
        int init;
    public:
        void PowerIntake();
};

class Op_LaunchDisk : public Op_PowerIntake{
    private:
        int init;
    public:
        void LaunchDisk();
};

class Op_SetPowerAmount : public Op_LaunchDisk{
    private:
        int init;
    public:
        void SetPowerAmount();
};

class Op_SetMotorType : public Op_SetPowerAmount {
    private:
        int init;
    public:
        void setMotorType();
};

namespace Op_Util{
    int sgn(double num);
    void stop();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void fullreset(double resetval, bool imu);
};