#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include "../00_Utils/PID_Controller.h"
#include "../00_Utils/df2_filter.h"
#include "../00_Utils/KincoDriver.h"

#define SLEW_COMPLETE_THRESH_POSN 1.5//0.08
#define SLEW_COMPLETE_THRESH_RATE 0.003
// #define SIDEREAL_RATE_DPS 0.004166667
#define DEFAULT_SLEW_MULT 64

typedef enum
{
    SLEWING_TO_POSN,
    MANUAL_SLEW,
    TRACKING_COMMAND,
    HOMING_IN_PROGRESS
} ControlMode_t;

class SlewDrive
{

private:
    const char *axisLabel;

    double positionFeedback_deg;
    double positionCommand_deg;
    double positionOffset_deg;
    double posnError;
    double rateFeedback_dps;
    double rateCommandFeedforward_dps;
    double manualRateCommand_dps;
    double rateRef_dps;
    double rateError;
    double combinedRateCmdSaturated_dps;
    bool isEnabled;
    double rateLim;
    bool drvAConnected;
    bool drvBConnected;
    // const PID_Controller *pid;
    std::unique_ptr<PID_Controller> pid;
    std::unique_ptr<DF2_IIR<double>> driveModelPtr;

    std::unique_ptr<KincoDriver> pDriveA;
    std::unique_ptr<KincoDriver> pDriveB;

    bool simModeEnabled;
    typedef enum
    {
        HOMING_IDLE,
        HOME_COMMAND_RECEIVED,
        PRE_HOME_ALIGNMENT,
        PREPARE_FOR_HOMING,
        HOMING_ACTIVE,
        HOMING_CLEANUP,
        HOMING_COMPLETE
    } axisHomingStatus_t;

    axisHomingStatus_t homingRoutineStatus;

    uint32_t alignmentCounter{0};

    bool updateAlignment();
    bool prepForHoming();
    void updatePositionError();
public:
    SlewDrive(const char *label, unsigned DriveA_ID, unsigned DriveB_ID, bool simMode = false);
    static bool initializeDriverBus(const char *devPath);
    bool connectToDrivers();
    void enable();
    void disable();
    void initializeStates();

    double getPositionCommand() { return std::fmod(positionCommand_deg, 360.0); }
    double getPositionFeedback();
    double getPositionState();
    double processPositionFeedback(double currPosn);

    double getVelocityCommand() { return rateCommandFeedforward_dps + rateRef_dps; }
    double getVelocityFeedback();
    double getVelocityState();

    void updateTrackCommands(double pcmd, double rcmd = 0.0);

    void abortSlew();
    void syncPosition(double sync_posn);
    bool isSlewComplete();
    void slowStop();
    // SlewDriveMode_t poll();
    void updateRateOffset(double rate);
    void updateManualRateCommand(double rate);
    void updateSlewRate(double slewRate);
    const char *getModeString();
    void updateControlLoops(double dt, ControlMode_t mode);
    static double mapSlewDriveCommandToMotors(double);
    double mapMotorPositionToSlewDrive(double motorPosn_deg);

    void startHoming();

    void serviceHomingRoutine();
    bool isHomingComplete();
    void resetHomingRoutine();
    void checkDriveStatus();

    void setSimulationMode(bool);
    bool getSimulationMode(){return simModeEnabled;}
    void simulate(double dt);

    std::vector<std::string> debugStrings;
};
