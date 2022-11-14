#pragma once

#include <vector>
#include <string>

#define SLEW_COMPLETE_THRESH 0.0005
#define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 64
class SlewDrive
{

private:
    const char *axisLabel;

    enum controlMode
    {
        POSITION_CONTROL,
        RATE_CONTROL
    };

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

    double getPositionCommand() { return positionCommand_deg; }
    double getPositionFeedback() { return positionFeedback_deg; }

    double getVelocityCommand() { return rateCommandOffset_dps+rateRef_dps; }
    double getVelocityFeedback() { return rateFeedback_dps; }

    void updateTrackCommands(double pcmd, double rcmd = 0.0);
    void abortSlew();
    void syncPosition(double posn);
    bool isSlewComplete();
    void slowStop(){abortSlew();}
    bool isStopped(){return rateFeedback_dps == 0;}
    // SlewDriveMode_t poll();
    void updateRateOffset(double rate);
    void updateSlewRate(double slewRate);
    const char *getModeString();
#if SIM_MODE_ENABLED
    void simulate(double dt);
#endif

    std::vector<std::string> debugStrings;
};