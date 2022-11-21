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
    std::unique_ptr<DF2_IIR<double>> driveModelPtr;

    std::unique_ptr<KinkoDriver> pDriveA;
    std::unique_ptr<KinkoDriver> pDriveB;

    bool simModeEnabled;

public:
    SlewDrive(const char *label, unsigned DriveA_ID, unsigned DriveB_ID, bool simMode = false);
    static bool initializeDriverBus(const char *devPath);
    bool connectToDrivers();
    void enable();
    void disable();
    void initializeStates();
    double getPositionCommand() { return std::fmod(positionCommand_deg, 360.0); }
    double getPositionFeedback();
    double processPositionFeedback(double currPosn);

    double getVelocityCommand() { return rateCommandOffset_dps + rateRef_dps; }
    double getVelocityFeedback();

    void updateTrackCommands(double pcmd, double rcmd = 0.0);

    void abortSlew();
    void syncPosition(double posn);
    bool isSlewComplete();
    void slowStop();
    bool isStopped() { return rateFeedback_dps == 0; }
    // SlewDriveMode_t poll();
    void updateRateOffset(double rate);
    void updateSlewRate(double slewRate);
    const char *getModeString();
    void updateControlLoops(double dt, ControlMode_t mode);
    static double mapSlewDriveCommandToMotors(double);
    double mapMotorPositionToSlewDrive(double motorPosn_deg);
    void simulate(double dt);

    std::vector<std::string> debugStrings;
};