#include "slew_drive.h"
#include <cmath>
#include <sstream>

/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////

SlewDrive::SlewDrive(const char *label)
{
    axisLabel = label;
    // Initialize state variables
    gotoAndStopCommandReceived = false;
    trackCommandUpdateReceived = false;
    abortCommandReceived = false;
    isEnabled = false;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommand_dps = 0.0;
    fastSlewRate = FAST_SLEW_DEFAULT_DPS;
    currentMode = INIT;
}

double SlewDrive::getPositionFeedback()
{
    return positionFeedback_deg;
}

void SlewDrive::gotoAndStop(double cmd)
{
    positionCommand_deg = cmd;
    gotoAndStopCommandReceived = true;
}

void SlewDrive::updatePositionCommand(double cmd)
{
    if (currentMode == SLEWING)
        positionCommand_deg = cmd;
}
void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    if (currentMode == TRACKING)
    {
        trackCommandUpdateReceived = true;
        positionCommand_deg = pcmd;
        rateCommand_dps = rcmd;
    }
}
void SlewDrive::abortSlew()
{

    abortCommandReceived = true;
}
void SlewDrive::syncPosition(double posn)
{
    positionFeedback_deg = posn;
}
bool SlewDrive::isSlewComplete()
{
    return false;
}
void SlewDrive::setSlewRate(double slewRate)
{
    fastSlewRate = slewRate;
}
void SlewDrive::enable()
{
    isEnabled = true;
}

#if SIM_MODE_ENABLED
void SlewDrive::simulate(double dt)
{
    double deltaPos = rateCommand_dps * dt;
    positionFeedback_deg += deltaPos;
    // if (positionFeedback_deg == 0.0) return;

    // std::stringstream ss;
    // ss << axisLabel << " Position Feedback: " << positionFeedback_deg << ", delta pos: " << deltaPos;
    // debugStrings.push_back(ss.str());
}
#endif

const char *SlewDrive::getModeString()
{
    switch (currentMode)
    {
    case INIT:
        return "INIT";
    case IDLE:
        return "IDLE";
    case STOPPING:
        return "STOPPING";
    case SLEWING:
        return "SLEWING";
    case TRACKING:
        return "TRACKING";
    case ERROR:
    default:
        return "ERROR";
    }
}
//////////////////////////////////////////////////////////////////////////
////////////////////// PRIVATE MEMBER FUNCTIONS //////////////////////////
//////////////////////////////////////////////////////////////////////////
SlewDriveMode_t SlewDrive::poll()
{
    SlewDriveMode_t nextMode;
    switch (currentMode)
    {
    case INIT:
        nextMode = initHandler();
        break;
    case IDLE:
        nextMode = idleHandler();
        break;
    case STOPPING:
        nextMode = stoppingHandler();
        break;
    case SLEWING:
        nextMode = slewingHandler();
        break;
    case TRACKING:
        nextMode = trackingHandler();
        break;
    case ERROR:
        nextMode = errorHandler();
        break;
    }
    currentMode = nextMode;
    return currentMode;
}

SlewDriveMode_t SlewDrive::initHandler()
{

    // 2. Establish communications with drivers
    // TODO

    return IDLE;
}
SlewDriveMode_t SlewDrive::idleHandler()
{
    SlewDriveMode_t nextState = IDLE;
    if (isEnabled)
    {
        if (gotoAndStopCommandReceived)
        {
            std::stringstream ss;
            ss << axisLabel << ": Switching to slew mode.";
            debugStrings.push_back(ss.str());

            nextState = SLEWING;
            gotoAndStopCommandReceived = false;
        }
        else
        {
            std::stringstream ss;
            ss << axisLabel << ": Goto command received while disabled.";
            debugStrings.push_back(ss.str());
        }
    }
    return nextState;
}
SlewDriveMode_t SlewDrive::stoppingHandler()
{
    rateCommand_dps = 0.0;
    // If rate feedback is zero -> idle, otherwise still stopping.
    return IDLE;
}
SlewDriveMode_t SlewDrive::slewingHandler()
{
    SlewDriveMode_t nextState = SLEWING;
    if (abortCommandReceived)
    {
        nextState = STOPPING;
    }
    else
    {
        double fastSlewThresh = fastSlewRate * 1.5;
        double posnError = positionCommand_deg - positionFeedback_deg;
        if (std::abs(posnError) < fastSlewThresh)
        {
            nextState = TRACKING;
        }
        else
        {
            int sign = std::signbit(posnError) ? -1 : 1;
            rateCommand_dps = fastSlewRate * sign;
        }
    }
    return nextState;
}

SlewDriveMode_t SlewDrive::trackingHandler()
{
    SlewDriveMode_t nextState = TRACKING;
    if (abortCommandReceived)
    {
        nextState = STOPPING;
    }
    else
    {
        rateCommand_dps = 0.0;
    }
    // Closed loop behavior goes here.
    return nextState;
}

SlewDriveMode_t SlewDrive::errorHandler()
{
    rateCommand_dps = 0.0;
    return ERROR;
}
