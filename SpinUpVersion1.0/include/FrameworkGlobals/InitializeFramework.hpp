#include "main.h"

extern unsigned short int globalAuton;
extern unsigned short int AutonFinalized;
extern unsigned short int SelectedAuton; // Auton choice

class Init_AutonSwitchMain{
    private:
        bool init;
    public:
        void ReceiveInput(long int time);
        void ReceiveInput_noLimit(long int time);
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
        void DisplayData();
        void DisplayCurrentAuton();
};