#include "slew_drive.h"
#include <cmath>
#include <sstream>
#include "../00_Utils/math_util.h"
/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////
#define MULT 10
constexpr double MAX_RATE_CMD = 0.25 * MULT;
constexpr double MIN_RATE_CMD = -0.25 * MULT;
double SLEW_DRIVE_MAX_SPEED_DPS = 500;

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
SlewDrive::SlewDrive(const char *label)
{
    axisLabel = label;
    // Initialize state variables
    isEnabled = false;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommandOffset_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
    rateLim = MAX_RATE_CMD;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    positionCommand_deg = pcmd;
    rateCommandOffset_dps = rcmd;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::abortSlew()
{
    positionCommand_deg = positionFeedback_deg;
    rateRef_dps = 0.0;
    rateCommandOffset_dps = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::syncPosition(double posn)
{
    rateCommandOffset_dps = 0.0;
    positionCommand_deg = posn;
    positionFeedback_deg = posn;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool SlewDrive::isSlewComplete()
{
    double posnError = positionCommand_deg - positionFeedback_deg;
    bool isComplete = (std::abs(posnError) <= SLEW_COMPLETE_THRESH);
    return isComplete;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateSlewRate(double slewRate)
{
    rateLim = slewRate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateRateOffset(double rate)
{
    rateCommandOffset_dps = rate;
}
void SlewDrive::enable()
{
    isEnabled = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
#if SIM_MODE_ENABLED

void SlewDrive::simulate(double dt, ControlMode_t mode)
{
    const double kp = 1.0;
    rateFeedback_dps = rateCommandOffset_dps;

    double posnError = positionCommand_deg - positionFeedback_deg;
    int errSign = sign(posnError);
    while (std::abs(posnError) > 180.0)
    {
        posnError -= 360.0 * errSign;
    }

    rateRef_dps = posnError * kp;
    // rateRef_dps = 0;

    double combinedRateCmd_dps{0};

    if (mode == POSITION_CONTROL)
    {
        combinedRateCmd_dps = saturate(rateRef_dps, -1 * rateLim, rateLim);
        // combinedRateCmd_dps = saturate(rateRef_dps + rateCommandOffset_dps, -1 * rateLim, rateLim);
    }
    else if (mode == RATE_CONTROL)
    {
        combinedRateCmd_dps = saturate(rateCommandOffset_dps, -1 * rateLim, rateLim);
    }

    double combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps, -1 * SLEW_DRIVE_MAX_SPEED_DPS, SLEW_DRIVE_MAX_SPEED_DPS);

    rateFeedback_dps = combinedRateCmd_dps;
    double deltaPos = rateFeedback_dps * dt;

    if ((std::abs(deltaPos) <= SLEW_COMPLETE_THRESH) && std::abs(rateCommandOffset_dps) == 0.0)
    {
        positionFeedback_deg = positionCommand_deg;
        rateCommandOffset_dps = 0;
        rateRef_dps = 0.0;
        rateFeedback_dps = 0.0;
    }
    else
    {
        positionFeedback_deg += deltaPos;
    }
}

#endif
