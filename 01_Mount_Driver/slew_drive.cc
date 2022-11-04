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
    activeCommand = NO_COMMAND;
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

void SlewDrive::slewToTrack(double cmd)
{
    positionCommand_deg = cmd;
    activeCommand = SLEW_TO_TRACK;
}

void SlewDrive::slewToStop(double cmd)
{
    positionCommand_deg = cmd;
    activeCommand = SLEW_TO_STOP;
}

void SlewDrive::updatePositionCommand(double cmd)
{
        positionCommand_deg = cmd;
}
void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    if (currentMode == TRACKING)
    {
        positionCommand_deg = pcmd;
        rateCommand_dps = rcmd;
    }
}
void SlewDrive::abortSlew()
{
    rateCommand_dps = 0.0;
    activeCommand = ABORT;
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
    case SLEWING_TO_TRACK:
        return "SLEWING_TO_TRACK";
    case SLEWING_TO_STOP:
        return "SLEWING_TO_STOP";
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
    case SLEWING_TO_TRACK:
        nextMode = slewingToTrackHandler();
        break;
    case SLEWING_TO_STOP:
        nextMode = slewingToStopHandler();
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
        switch (activeCommand)
        {
        case SLEW_TO_TRACK:
            nextState = SLEWING_TO_TRACK;
            break;
        case SLEW_TO_STOP:
            nextState = SLEWING_TO_STOP;
            break;
        case ABORT:
        case NO_COMMAND:
        default:
            break;
        }
        activeCommand = NO_COMMAND;
    }
    else
    {
        // Throw exception
    }
    return nextState;
}

SlewDriveMode_t SlewDrive::slewingToTrackHandler()
{
    SlewDriveMode_t nextState = SLEWING_TO_TRACK;

    if (isEnabled)
    {
        switch (activeCommand)
        {
        case SLEW_TO_STOP:
            nextState = SLEWING_TO_STOP;
            break;
        case ABORT:
            nextState = STOPPING;
            break;
        case SLEW_TO_TRACK:
        case NO_COMMAND:
        default:
        {
            double fastSlewThresh = fastSlewRate * 1.5 * 0.1;
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
        break;
        }
        activeCommand = NO_COMMAND;
    }
    else
    {
        // Throw exception
    }

    return nextState;
}

SlewDriveMode_t SlewDrive::trackingHandler()
{
    SlewDriveMode_t nextState = TRACKING;

    if (isEnabled)
    {
        switch (activeCommand)
        {
        case SLEW_TO_STOP:
            nextState = SLEWING_TO_STOP;
            break;
        case ABORT:
            nextState = STOPPING;
            break;
        case SLEW_TO_TRACK:
            // Reset integrators?
            nextState = SLEWING_TO_TRACK;
            break;
        case NO_COMMAND:
        default:
        {
            double posnError = positionCommand_deg - positionFeedback_deg;
            int sign = std::signbit(posnError) ? -1 : 1;
            rateCommand_dps = 0.004166667 * sign;
        }
        break;
        }
        activeCommand = NO_COMMAND;
    }
    else
    {
        // Throw exception
    }

    return nextState;
}

SlewDriveMode_t SlewDrive::slewingToStopHandler()
{
    SlewDriveMode_t nextState = SLEWING_TO_STOP;
    if (isEnabled)
    {
        switch (activeCommand)
        {
        case SLEW_TO_STOP:
            nextState = SLEWING_TO_STOP;
            break;
        case ABORT:
            nextState = STOPPING;
            break;
        case SLEW_TO_TRACK:
        case NO_COMMAND:
        default:
        {
            double fastSlewThresh = fastSlewRate * 1.5 * 0.1;
            double posnError = positionCommand_deg - positionFeedback_deg;
            if (std::abs(posnError) < fastSlewThresh)
            {
                nextState = STOPPING;
            }
            else
            {
                int sign = std::signbit(posnError) ? -1 : 1;
                rateCommand_dps = fastSlewRate * sign;
            }
        }
        break;
        }
        activeCommand = NO_COMMAND;
    }
    else
    {
        // Throw exception
    }

    return nextState;
}

SlewDriveMode_t SlewDrive::stoppingHandler()
{
    rateCommand_dps = 0.0;
    // If rate feedback is zero -> idle, otherwise still stopping.
    return IDLE;
}

SlewDriveMode_t SlewDrive::errorHandler()
{
    rateCommand_dps = 0.0;
    return ERROR;
}
