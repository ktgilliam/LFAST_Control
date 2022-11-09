#pragma once

#include <vector>
#include <string>
#include <cmath>

#define SLEW_COMPLETE_THRESH 0.0000005
#define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 64

// These values are in radians per second
static constexpr double SIDEREALRATE{(2 * M_PI / 86164.09065)};
static constexpr double MAX_SPEED{500.0};
static constexpr double LOW_SPEED_MARGIN{128.0 * SIDEREALRATE};

class SlewDrive
{

private:
    const char *axisLabel;

    typedef enum 
    {
        STOPPED,
        POSITION_CONTROL,
        RATE_CONTROL,

    } ControlMode_t;

    double positionFeedback_deg;
    double positionCommand_deg;
    double rateFeedback_dps;
    // double rateCommandOffset_dps;
    double combinedRateCmd_dps;
    double rateRef_dps;
    double fastSlewRate;
    bool isEnabled;
    ControlMode_t mode;

public:
    SlewDrive(const char *);
    void enable();

    double getPositionFeedback() { return positionFeedback_deg; }
    double getVelocityFeedback() { return rateFeedback_dps; }
    double getPositionCommand() { return positionCommand_deg; }
        double getVelocityCommand() { return combinedRateCmd_dps; }
    // double getVelocityCommand() { return rateCommandOffset_dps + rateRef_dps; }

    void updateTrackCommands(double pcmd, double rcmd = 0.0);
    void abortSlew();
    void setPosition(double posn);
    bool isSlewComplete();
    void slowStop() {abortSlew();}
    bool isStopped() { return rateFeedback_dps == 0; }
    // SlewDriveMode_t poll();
    void updateRateOffset(double slewRate);
    const char *getModeString();
#if SIM_MODE_ENABLED
    void simulate(double dt);
#endif

    std::vector<std::string> debugStrings;
};