/*******************************************************************************
 Copyright(c) 2017 Jasem Mutlaq. All rights reserved.

 Driver for using TheSky6 Pro Scripted operations for mounts via the TCP server.
 While this technically can operate any mount connected to the TheSky6 Pro, it is
 intended for LFAST_Mount mounts control.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include "indiguiderinterface.h"
#include "inditelescope.h"
#include "inditimer.h"
#include "indipropertyswitch.h"

#include "../00_Utils/lfast_comms.h"

union ByteConverter
{
    unsigned long INT;
    double DOUBLE;
    unsigned char BYTES[8];
};

#define MOUNT_PARKING_ENABLED 1
#define MOUNT_GUIDER_ENABLED 0
#define MOUNT_GOTO_ENABLED 1
// #define MOUNT_GOTO_ENABLED
#define TRACK_SOLAR_ENABLED 0
#define TRACK_LUNAR_ENABLED 0
#define TRACK_CUSTOM_ENABLED 0
#define TRACK_ALT_AZ_ENABLED 0
#define NUM_TRACK_MODES (1 + TRACK_SOLAR_ENABLED + TRACK_LUNAR_ENABLED + TRACK_ALT_AZ_ENABLED + TRACK_CUSTOM_ENABLED)

#define AZ_EL_JOG_ENABLED 1
#define NUM_JOG_MODES (1 + AZ_EL_JOG_ENABLED)

enum
{
    RA_AXIS,
    DEC_AXIS,
    NUM_SR_AXES
};
enum
{
    ALT_AXIS,
    AZ_AXIS,
    NUM_MECH_AXES
};
enum
{
    LFAST_NORTH,
    LFAST_SOUTH,
    LFAST_EAST,
    LFAST_WEST,
    // LFAST_ALT,
    // LFAST_AZ
};

#define JOG_RATE_MIN 0
#define JOG_RATE_MAX 600
#define JOG_RATE_STEP 60
#define JOG_RATE_VALUE 30

#if MOUNT_GUIDER_ENABLED
class LFAST_Mount : public INDI::Telescope, public INDI::GuiderInterface
#else
class LFAST_Mount : public INDI::Telescope
#endif
{
public:
    LFAST_Mount();
    virtual ~LFAST_Mount() = default;

    virtual const char *getDefaultName() override;
    virtual bool Handshake() override;
    virtual bool ReadScopeStatus() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

protected:
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    // bool MoveAltAz(INDI_HO_AXIS dir, TelescopeMotionCommand command);

    virtual bool Abort() override;

    virtual bool updateTime(ln_date *utc, double utc_offset) override;

#if MOUNT_PARKING_ENABLED
    // Parking
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;
    virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;
    virtual bool Park() override;
    virtual bool UnPark() override;
#endif
    virtual bool Goto(double, double) override;
    virtual bool Sync(double ra, double dec) override;

    // Tracking
    virtual bool SetTrackMode(uint8_t mode) override;
    virtual bool SetTrackRate(double raRate, double deRate) override;
    virtual bool SetTrackEnabled(bool enabled) override;

#if MOUNT_GUIDER_ENABLED
    // Guiding
    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;
    // these all call these two functions
    IPState GuideNS(int32_t ms);
    IPState GuideWE(int32_t ms);
#endif
private:
    void mountSim();
    bool getMountAltAz();
    bool isSlewComplete();

    // bool sendMountOKCommand(const char *command, const char *errorMessage, uint8_t timeout = 3);
    bool sendMountOKCommand(LFAST::MessageGenerator &cmdMsg, const char *errorMessage, uint8_t timeout = 3);
#if MOUNT_PARKING_ENABLED
    bool isMountParked();
#endif
    bool isMountTracking();
    bool startOpenLoopMotion(uint8_t motion, uint16_t rate);
    bool stopOpenLoopMotion();
    bool setTheSkyTracking(bool enable, bool isSidereal, double raRate, double deRate);

    double getCurrentRa();
    double getCurrentDec();
    double getTargetRa();
    double getTargetDec();
    double getDeltaRa();
    double getDeltaDec();

    void setTargetRaDec(double ra, double dec);

    void AltAzToRaDec(double alt, double az, double *ra, double *dec);
    void RaDecToAltAz(double ra, double dec, double *alt, double *az);
    void altAzToHADec(double alt, double az, double *ha, double *dec);
    // Homing
    bool findHome();

    void printScopeMode();
    // double currentRA{0};
    // double currentDEC{90};
    // double targetRA{0};
    // double targetDEC{0};

    double currentALT{0};
    double currentAZ{90};
    double targetALT{0};
    double targetAZ{0};

    unsigned int DBG_SCOPE{0};

    // Jog Rate
    enum
    {
        JOG_MODE_NSEW,
        JOG_MODE_ALT_AZ,
    };
    // ISwitch JogModeS[NUM_JOG_MODES];
    // ISwitchVectorProperty JogModeSP;
    INDI::PropertySwitch JogModeSP{2};

    INumber JogRateNSEW_N[NUM_SR_AXES];
    INumberVectorProperty JogRateNSEW_NP;

    INumber JogRateAltAz_N[NUM_MECH_AXES];
    INumberVectorProperty JogRateAltAz_NP;

#if MOUNT_GUIDER_ENABLED
    // Guide Rate
    INumber GuideRateN[2];
    INumberVectorProperty GuideRateNP;
#endif
    // Jogging Mode

    // A switch for Alt motion
    ISwitch MovementAltS[2];
    ISwitchVectorProperty MovementAltSP;
    // A switch for Az motion
    ISwitch MovementAzS[2];
    ISwitchVectorProperty MovementAzSP;

// Tracking Mode
// #if NUM_TRACK_MODES > 1
    ISwitch TrackModeS[NUM_TRACK_MODES];
    ISwitchVectorProperty TrackModeSP;
// #endif
    // Homing
    ISwitchVectorProperty HomeSP;
    ISwitch HomeS[1];

    // Timers
    INDI::Timer m_NSTimer;
    INDI::Timer m_WETimer;

    // Tracking Rate
    //    INumber TrackRateN[2];
    //    INumberVectorProperty TrackRateNP;
};
