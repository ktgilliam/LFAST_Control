#include "slew_drive.h"

SlewDrive::SlewDrive()
{

    currentMode = INIT;
}

double SlewDrive::getPositionFeedback()
{
    return positionFeedback_deg;
}

void SlewDrive::gotoAndStop(double cmd)
{
    gotoAndStopCommandReceived = true;
}
void SlewDrive::gotoAndTrack(double pcmd, double rcmd)
{
    gotoAndTrackCommandReceived = true;
}
void SlewDrive::updatePositionCommand(double cmd)
{

}
void SlewDrive::updateTrackCommand(double rcmd)
{
}
void SlewDrive::abortSlew()
{
}
void SlewDrive::syncPosition(double posn)
{
}

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
    case STOPPED:
        nextMode = stoppedHandler();
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
    // 1. Initialize state variables
    gotoAndStopCommandReceived = false;
    gotoAndTrackCommandReceived = false;
    trackCommandUpdateReceived = false;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommand_dps = 0.0;

    // 2. Establish communications with drivers
    // TODO

    return IDLE;
}
SlewDriveMode_t SlewDrive::idleHandler()
{
    return IDLE;
}
SlewDriveMode_t SlewDrive::stoppedHandler()
{
    return IDLE;
}
SlewDriveMode_t SlewDrive::slewingHandler()
{
    return IDLE;
}
SlewDriveMode_t SlewDrive::trackingHandler()
{
    return IDLE;
}
SlewDriveMode_t SlewDrive::errorHandler()
{
    return IDLE;
}