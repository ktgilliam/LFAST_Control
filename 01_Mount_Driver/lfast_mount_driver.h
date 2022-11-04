
#pragma once

#include "indiguiderinterface.h"
#include "inditelescope.h"
#include "indipropertytext.h"
#include "indipropertynumber.h"
#include "alignment/AlignmentSubsystemForDrivers.h"

#include "slew_drive.h"

#define SIDEREAL_RATE_DPS 0.004166667

namespace LFAST
{
    const double slewspeeds[] = {1.0, 2.0, 4.0, 8.0, 32.0, 64.0, 128.0, 256.0, 512.0};
constexpr unsigned int NUM_SLEW_SPEEDS = sizeof(slewspeeds) / sizeof(double);
constexpr unsigned int DEFAULT_SLEW_IDX = NUM_SLEW_SPEEDS - 1;
}
class LFAST_Mount : public INDI::Telescope, public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
{
public:
    LFAST_Mount();
    virtual ~LFAST_Mount();

protected:
    /** \brief Called to initialize basic properties required all the time */
    virtual bool initProperties() override;
    /** \brief Called when connected state changes, to add/remove properties */
    virtual bool updateProperties() override;

    bool Abort() override;
    bool Connect() override;
    bool Disconnect() override;
    bool Handshake() override;

    const char *getDefaultName() override;
    bool Goto(double ra, double dec) override;
    bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                   char *formats[], char *names[], int n) override;
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    bool ReadScopeStatus() override;
    bool Sync(double ra, double dec) override;
    bool updateLocation(double latitude, double longitude, double elevation) override;

    void TimerHit() override;
    void mountSim();

    // Parking
    virtual bool Park() override;
    virtual bool UnPark() override;
    // virtual bool SetCurrentPark() override;
    // virtual bool SetDefaultPark() override;
    // virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;

    void updateTrackingTarget(double ra, double dec);
    bool SetSlewRate(int index) override;

    INDI::IHorizontalCoordinates getTrackingTargetAltAzPosition();

private:
    unsigned int DBG_SCOPE{0};

    SlewDrive *AltitudeAxis;
    SlewDrive *AzimuthAxis;

    // Tracking
    INDI::IEquatorialCoordinates CurrentTrackingTarget{0, 0};

    // Tracing in timer tick
    int TraceThisTickCount{0};
    bool TraceThisTick{false};

    unsigned int DBG_SIMULATOR{0};

    INDI::PropertyText NtpServerTP{1};
    INDI::PropertyNumber AzAltCoordsNP{2};
    INDI::PropertySwitch MountSlewRateSP{LFAST::NUM_SLEW_SPEEDS};
    
};

const std::string getDirString(INDI_DIR_NS dir)
{
    if (dir == DIRECTION_NORTH)
        return "DIRECTION_NORTH";
    else if (dir == DIRECTION_SOUTH)
        return "DIRECTION_SOUTH";
    else
        return "DIRECTION_ERROR";
}

const std::string getDirString(INDI_DIR_WE dir)
{
    if (dir == DIRECTION_WEST)
        return "DIRECTION_WEST";
    else if (dir == DIRECTION_EAST)
        return "DIRECTION_EAST";
    else
        return "DIRECTION_ERROR";
}