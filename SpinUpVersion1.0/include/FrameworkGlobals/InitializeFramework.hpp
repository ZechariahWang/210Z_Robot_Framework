#include "main.h"

extern unsigned short int globalAuton;

class Init_AutonSwitchMain{
    private:
        bool init;
    public:
        void ReceiveInput(long int time);
};

class ResetSensors : public Init_AutonSwitchMain{
    private:
        bool init;
    public:
        void ResetAllPrimarySensors();
};

class FinalizeAuton : public ResetSensors{
    private:
        bool init;
    public:
        void SelectAuton();
};