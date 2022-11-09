#include "slew_drive.h"
#include <cmath>
#include <sstream>
#include "../00_Utils/math_util.h"
/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////

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
    combinedRateCmd_dps = 0.0;
    mode = STOPPED;
}

void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    mode = POSITION_CONTROL;
    positionCommand_deg = pcmd;
    rateCommandOffset_dps = rcmd;
}
void SlewDrive::abortSlew()
{
    positionCommand_deg = positionFeedback_deg;
    rateRef_dps = 0.0;
    rateCommandOffset_dps = 0.0;
}

void SlewDrive::setPosition(double posn)
{
    rateCommandOffset_dps = 0.0;
    positionCommand_deg = posn;
    positionFeedback_deg = posn;
}
bool SlewDrive::isSlewComplete()
{
    double posnError = positionCommand_deg - positionFeedback_deg;
    bool isComplete = (std::abs(posnError) <= SLEW_COMPLETE_THRESH);
    return isComplete;
}

void SlewDrive::updateRateOffset(double slewRate)
{
    mode = RATE_CONTROL;
    rateCommandOffset_dps = slewRate;
}
bool SlewDrive::setSlewRate(double rate)
{
    double newSlewRate = std::abs(rate) < MAX_SPEED ? rate : MAX_SPEED;
    MAX_RATE_CMD = newSlewRate;
    MIN_RATE_CMD = -1 * MAX_RATE_CMD;
    return true;
}
// bool SlewDrive::setSlewRateMultiplier(double mult)
// {
//     MAX_RATE_CMD = 1.2727e-6 * mult;
//     MIN_RATE_CMD = -1 * MAX_RATE_CMD;
//     return true;
// }

void SlewDrive::enable()
{
    isEnabled = true;
}

#if SIM_MODE_ENABLED
const double kp = 1.0;

void SlewDrive::simulate(double dt)
{

    // rateFeedback_dps = rateCommand_dps;

    double posnError = positionCommand_deg - positionFeedback_deg;
    int errSign = sign(posnError);
    while (std::abs(posnError) > 180.0)
    {
        posnError -= 360.0 * errSign;
    }

    switch (mode)
    {
    case STOPPED:
        rateCommandOffset_dps = 0.0;
        rateRef_dps = 0.0;
        combinedRateCmd_dps = 0.0;
        break;
    case RATE_CONTROL:
        rateRef_dps = 0.0;
        combinedRateCmd_dps = saturate(rateCommandOffset_dps, MIN_RATE_CMD, MAX_RATE_CMD);
        break;
    case POSITION_CONTROL:
        rateRef_dps = posnError * kp;
        combinedRateCmd_dps = saturate(rateRef_dps + rateCommandOffset_dps, MIN_RATE_CMD, MAX_RATE_CMD);
        break;
    }

    // Temporary simulated rate feedback :
    rateFeedback_dps = combinedRateCmd_dps;
    double deltaPos = rateFeedback_dps * dt;

    if ((std::abs(deltaPos) <= SLEW_COMPLETE_THRESH) && std::abs(rateCommandOffset_dps) == 0.0)
    {
        positionFeedback_deg = positionCommand_deg;
        rateCommandOffset_dps = 0;
        rateRef_dps = 0.0;
        rateFeedback_dps = 0.0;
        mode = STOPPED;
    }
    else
    {
        positionFeedback_deg += deltaPos;
    }
}

#endif