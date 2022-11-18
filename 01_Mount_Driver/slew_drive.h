#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include "../00_Utils/PID_Controller.h"
#include "../00_Utils/df2_filter.h"
#include "../00_Utils/KinkoDriver.h"

#define SLEW_COMPLETE_THRESH_POSN 0.05
#define SLEW_COMPLETE_THRESH_RATE 0.003
// #define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 64

typedef enum
{
    POSN_CONTROL_ONLY,
    POSN_AND_RATE_CONTROL
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
    double combinedRateCmdSaturated_dps;
    bool isEnabled;
    double rateLim;
    // const PID_Controller *pid;
    std::unique_ptr<PID_Controller> pid;
    std::unique_ptr<KinkoDriver> pDriveA;
    std::unique_ptr<KinkoDriver> pDriveB;

#if SIM_MODE_ENABLED
    std::unique_ptr<DF2_IIR<double>> driveModelPtr;
    // DF2_IIR<double> *driveModelPtr;
#endif
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
    void updateControlCommands(double dt, ControlMode_t mode);
    double mapSlewDriveCommandToMotors();
#if SIM_MODE_ENABLED
    void simulate(double dt);
#endif

    std::vector<std::string> debugStrings;
};