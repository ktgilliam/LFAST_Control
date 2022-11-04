#pragma once

#include <vector>
#include <string>
typedef enum
{
    INIT,
    IDLE,
    STOPPING,
    SLEWING,
    TRACKING,
    ERROR
} SlewDriveMode_t;

#define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 512
class SlewDrive
{

private:
    const char *axisLabel;

    enum controlMode
    {
        POSITION_CONTROL,
        RATE_CONTROL
    };

    static constexpr double FAST_SLEW_DEFAULT_DPS = SIDEREAL_RATE_DPS * DEFAULT_SLEW_MULT;

    bool gotoCommandReceived;
    bool trackCommandUpdateReceived;
    bool abortCommandReceived;

    double positionFeedback_deg;
    double positionCommand_deg;
    double rateCommand_dps;
    double fastSlewRate;
    bool isEnabled;
    // bool stopOnSlewComplete;

    SlewDriveMode_t currentMode;

    SlewDriveMode_t initHandler();
    SlewDriveMode_t idleHandler();
    SlewDriveMode_t stoppingHandler();
    SlewDriveMode_t slewingHandler();
    SlewDriveMode_t trackingHandler();
    SlewDriveMode_t errorHandler();

public:
    SlewDrive(const char*);
    void enable();

    double getPositionFeedback();
    void gotoAndStop(double cmd);
    void updatePositionCommand(double cmd);
    void updateTrackCommands(double pcmd, double rcmd);
    void abortSlew();
    void syncPosition(double posn);
    bool isSlewComplete();
    SlewDriveMode_t poll();
    void setSlewRate(double slewRate);
    const char* getModeString();
#if SIM_MODE_ENABLED
    void simulate(double dt);
#endif

    std::vector<std::string> debugStrings;
};