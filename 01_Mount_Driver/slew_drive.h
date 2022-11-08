#pragma once

#include <vector>
#include <string>
#include <cmath>

#define SLEW_COMPLETE_THRESH 0.0005
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

    enum controlMode
    {
        POSITION_CONTROL,
        RATE_CONTROL
    };

    double positionFeedback_deg;
    double positionCommand_deg;
    double rateFeedback_dps;
    double rateCommand_dps;
    double rateRef_dps;
    double fastSlewRate;
    bool isEnabled;

public:
    SlewDrive(const char *);
    void enable();

    double getPositionFeedback() { return positionFeedback_deg; }
    double getVelocityFeedback() { return rateFeedback_dps; }
    double getPositionCommand() { return positionCommand_deg; }
    double getVelocityCommand() { return rateCommand_dps + rateRef_dps; }

    void updateTrackCommands(double pcmd, double rcmd = 0.0);
    void abortSlew();
    void setPosition(double posn);
    bool isSlewComplete();
    void slowStop() {}
    bool isStopped() { return rateFeedback_dps == 0; }
    // SlewDriveMode_t poll();
    void slew(double slewRate);
    const char *getModeString();
#if SIM_MODE_ENABLED
    void simulate(double dt);
#endif

    std::vector<std::string> debugStrings;
};