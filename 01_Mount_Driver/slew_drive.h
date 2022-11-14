#pragma once

#include <vector>
#include <string>
#include <cmath>

#define SLEW_COMPLETE_THRESH 0.0005
#define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 64

typedef enum
{
    POSITION_CONTROL,
    RATE_CONTROL
} ControlMode_t;

class SlewDrive
{

private:
    const char *axisLabel;

    double positionFeedback_deg;
    double positionCommand_deg;
    double rateFeedback_dps;
    double rateCommandOffset_dps;
    double rateRef_dps;
    bool isEnabled;
    double rateLim;

public:
    SlewDrive(const char *);
    void enable();

    double getPositionCommand() { return std::fmod(positionCommand_deg, 360.0); }
    double getPositionFeedback() { return std::fmod(positionFeedback_deg, 360.0); }

    double getVelocityCommand() { return rateCommandOffset_dps + rateRef_dps; }
    double getVelocityFeedback() { return rateFeedback_dps; }

    void updateTrackCommands(double pcmd, double rcmd = 0.0);
    
    void abortSlew();
    void syncPosition(double posn);
    bool isSlewComplete();
    void slowStop() { abortSlew(); }
    bool isStopped() { return rateFeedback_dps == 0; }
    // SlewDriveMode_t poll();
    void updateRateOffset(double rate);
    void updateSlewRate(double slewRate);
    const char *getModeString();
#if SIM_MODE_ENABLED
    void simulate(double dt, ControlMode_t mode);
#endif

    std::vector<std::string> debugStrings;
};