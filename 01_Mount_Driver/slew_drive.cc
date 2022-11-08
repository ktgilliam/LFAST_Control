#include "slew_drive.h"
#include <cmath>
#include <sstream>
#include "../00_Utils/math_util.h"
/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////
#define MULT 10
constexpr double MAX_RATE_CMD = 0.25*MULT;
constexpr double MIN_RATE_CMD = -0.25*MULT;

SlewDrive::SlewDrive(const char *label)
{
    axisLabel = label;
    // Initialize state variables
    isEnabled = false;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommand_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
}


void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    positionCommand_deg = pcmd;
    rateCommand_dps = rcmd;
}
void SlewDrive::abortSlew()
{
    positionCommand_deg = positionFeedback_deg;
    rateRef_dps = 0.0;
    rateCommand_dps = 0.0;
}

void SlewDrive::setPosition(double posn)
{
    rateCommand_dps = 0.0;
    positionCommand_deg = posn;
    positionFeedback_deg = posn;
}
bool SlewDrive::isSlewComplete()
{
    double posnError = positionCommand_deg - positionFeedback_deg;
    bool isComplete = (std::abs(posnError) <= SLEW_COMPLETE_THRESH );
    return isComplete;
}

void SlewDrive::slew(double slewRate)
{
    fastSlewRate = slewRate;
}
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
    while(std::abs(posnError) > 180.0)
    {
        posnError -= 360.0*errSign;
    }

    rateRef_dps = posnError * kp; 
    double combinedRateCmd_dps = saturate(rateRef_dps + rateCommand_dps, MIN_RATE_CMD, MAX_RATE_CMD);

    rateFeedback_dps = combinedRateCmd_dps;
    double deltaPos = rateFeedback_dps * dt;

    if ((std::abs(deltaPos) <= SLEW_COMPLETE_THRESH) && std::abs(rateCommand_dps) == 0.0)
    {
        positionFeedback_deg = positionCommand_deg;
        rateCommand_dps = 0;
        rateRef_dps = 0.0;
        rateFeedback_dps = 0.0;
    }
    else
    {
        positionFeedback_deg += deltaPos;
    }
}

#endif

// const char *SlewDrive::getModeString()
// {
//     switch (currentMode)
//     {Goto(
//     case INIT:
//         return "INIT";
//     case IDLE:
//         return "IDLE";
//     case STOPPING:
//         return "STOPPING";
//     case SLEWING_TO_TRACK:
//         return "SLEWING_TO_TRACK";
//     case SLEWING_TO_STOP:
//         return "SLEWING_TO_STOP";
//     case TRACKING:
//         return "TRACKING";
//     case ERROR:
//     default:
//         return "ERROR";
//     }
// }
//////////////////////////////////////////////////////////////////////////
////////////////////// PRIVATE MEMBER FUNCTIONS //////////////////////////
//////////////////////////////////////////////////////////////////////////
// SlewDriveMode_t SlewDrive::poll()
// {
//     SlewDriveMode_t nextMode;
//     switch (currentMode)
//     {
//     case INIT:
//         nextMode = initHandler();
//         break;
//     case IDLE:
//         nextMode = idleHandler();
//         break;
//     case STOPPING:
//         nextMode = stoppingHandler();
//         break;
//     case SLEWING_TO_TRACK:
//         nextMode = slewingToTrackHandler();
//         break;
//     case SLEWING_TO_STOP:
//         nextMode = slewingToStopHandler();
//         break;
//     case TRACKING:
//         nextMode = trackingHandler();
//         break;
//     case ERROR:
//         nextMode = errorHandler();
//         break;
//     }
//     currentMode = nextMode;
//     return currentMode;
// }

// SlewDriveMode_t SlewDrive::initHandler()
// {

//     // 2. Establish communications with drivers
//     // TODO

//     return IDLE;
// }

// SlewDriveMode_t SlewDrive::idleHandler()
// {
//     SlewDriveMode_t nextState = IDLE;
//     if (isEnabled)
//     {
//         switch (activeCommand)
//         {
//         case SLEW_TO_TRACK:
//             nextState = SLEWING_TO_TRACK;
//             break;
//         case SLEW_TO_STOP:
//             nextState = SLEWING_TO_STOP;
//             break;
//         case ABORT:
//         case NO_COMMAND:
//         default:
//             break;
//         }
//         activeCommand = NO_COMMAND;
//     }
//     else
//     {
//         // Throw exception
//     }
//     return nextState;
// }

// SlewDriveMode_t SlewDrive::slewingToTrackHandler()
// {
//     SlewDriveMode_t nextState = SLEWING_TO_TRACK;

//     if (isEnabled)
//     {
//         switch (activeCommand)
//         {
//         case SLEW_TO_STOP:
//             nextState = SLEWING_TO_STOP;
//             break;
//         case ABORT:
//             nextState = STOPPING;
//             break;
//         case SLEW_TO_TRACK:
//         case NO_COMMAND:
//         default:
//         {
//             double fastSlewThresh = fastSlewRate * 1.5 * 0.1;
//             double posnError = positionCommand_deg - positionFeedback_deg;
//             if (std::abs(posnError) < fastSlewThresh)
//             {
//                 nextState = TRACKING;
//             }
//             else
//             {
//                 int sign = std::signbit(posnError) ? -1 : 1;
//                 rateCommand_dps = fastSlewRate * sign;
//             }
//         }
//         break;
//         }
//         activeCommand = NO_COMMAND;
//     }
//     else
//     {
//         // Throw exception
//     }

//     return nextState;
// }

// SlewDriveMode_t SlewDrive::trackingHandler()
// {
//     SlewDriveMode_t nextState = TRACKING;

//     if (isEnabled)
//     {
//         switch (activeCommand)
//         {
//         case SLEW_TO_STOP:
//             nextState = SLEWING_TO_STOP;
//             break;
//         case ABORT:
//             nextState = STOPPING;
//             break;
//         case SLEW_TO_TRACK:
//             // Reset integrators?
//             nextState = SLEWING_TO_TRACK;
//             break;
//         case NO_COMMAND:
//         default:
//         {
//             // double posnError = positionCommand_deg - positionFeedback_deg;
//             // int sign = std::signbit(posnError) ? -1 : 1;
//             // rateCommand_dps = 0.004166667 * sign;
//         }
//         break;
//         }
//         activeCommand = NO_COMMAND;
//     }
//     else
//     {
//         // Throw exception
//     }

//     return nextState;
// }

// SlewDriveMode_t SlewDrive::slewingToStopHandler()
// {
//     SlewDriveMode_t nextState = SLEWING_TO_STOP;
//     if (isEnabled)
//     {
//         switch (activeCommand)
//         {
//         case SLEW_TO_STOP:
//             nextState = SLEWING_TO_STOP;
//             break;
//         case ABORT:
//             nextState = STOPPING;
//             break;
//         case SLEW_TO_TRACK:
//         case NO_COMMAND:
//         default:
//         {
//             double fastSlewThresh = fastSlewRate * 1.5 * 0.1;
//             double posnError = positionCommand_deg - positionFeedback_deg;
//             if (std::abs(posnError) < fastSlewThresh)
//             {
//                 nextState = STOPPING;
//             }
//             else
//             {
//                 int sign = std::signbit(posnError) ? -1 : 1;
//                 rateCommand_dps = fastSlewRate * sign;
//             }
//         }
//         break;
//         }
//         activeCommand = NO_COMMAND;
//     }
//     else
//     {
//         // Throw exception
//     }

//     return nextState;
// }

// SlewDriveMode_t SlewDrive::stoppingHandler()
// {
//     rateCommand_dps = 0.0;
//     // If rate feedback is zero -> idle, otherwise still stopping.
//     return IDLE;
// }

// SlewDriveMode_t SlewDrive::errorHandler()
// {
//     rateCommand_dps = 0.0;
//     return ERROR;
// }
