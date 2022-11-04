#pragma once

typedef enum
{
    INIT,
    IDLE,
    STOPPED,
    SLEWING,
    TRACKING,
    ERROR
} SlewDriveMode_t;

class SlewDrive
{

private:
    enum controlMode
    {
        POSITION_CONTROL,
        RATE_CONTROL
    };

    bool gotoAndStopCommandReceived;
    bool gotoAndTrackCommandReceived;
    bool trackCommandUpdateReceived;

    double positionFeedback_deg;
    double positionCommand_deg;
    double rateCommand_dps;

    SlewDriveMode_t currentMode;

    SlewDriveMode_t initHandler();
    SlewDriveMode_t idleHandler();
    SlewDriveMode_t stoppedHandler();
    SlewDriveMode_t slewingHandler();
    SlewDriveMode_t trackingHandler();
    SlewDriveMode_t errorHandler();

public:
    SlewDrive();
    

    double getPositionFeedback();
    void gotoAndStop(double cmd);
    void gotoAndTrack(double pcmd, double rcmd);
    void updatePositionCommand(double cmd);
    void updateTrackCommand(double rcmd);
    void abortSlew();
    void syncPosition(double posn);
    SlewDriveMode_t poll();
};