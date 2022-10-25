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





#define AZ_EL_JOG_ENABLED 1
#define NUM_JOG_MODES (1 + AZ_EL_JOG_ENABLED)


#define LFAST_TIMEOUT 3 /* Timeout in seconds */
#define LFAST_MOUNT_HANDSHAKE_TIMEOUT 2
#define LFAST_HOMING_TIMEOUT 10

namespace LFAST
{
enum
{
    RA_AXIS,
    DEC_AXIS,
    NUM_AXES
};
enum
{
    NORTH,
    SOUTH,
    EAST,
    WEST
};

#define TRACK_SOLAR_ENABLED 0
#define TRACK_LUNAR_ENABLED 0
#define TRACK_CUSTOM_ENABLED 0
#define TRACK_ALT_AZ_ENABLED 0
#define NUM_TRACK_MODES (1 + TRACK_SOLAR_ENABLED + TRACK_LUNAR_ENABLED + TRACK_ALT_AZ_ENABLED + TRACK_CUSTOM_ENABLED)
enum
{
    TRACK_SIDEREAL = 0,
    TRACK_SOLAR = 1,
    TRACK_LUNAR = 2,
    TRACK_ALT_AZ = 3,
    TRACK_CUSTOM = 4,
};

typedef enum 
{
    SLEW_THEN_TRACK = 0,
    SLEW_ONLY = 1,
    SLEW_THEN_SYNC = 2
} SLEW_MODE;

}
#define JOG_RATE_MIN 0
#define JOG_RATE_MAX 600
#define JOG_RATE_STEP 60
#define JOG_RATE_VALUE 30

class LFAST_Mount : public INDI::Telescope, public INDI::GuiderInterface
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

        // Parking
        virtual bool SetCurrentPark() override;
        virtual bool SetDefaultPark() override;
        virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;
        virtual bool Park() override;
        virtual bool UnPark() override;
        virtual bool Goto(double, double) override;
        virtual bool Sync(double ra, double dec) override;

        // Tracking
        // virtual bool SetTrackMode(uint8_t mode) override;
        // virtual bool SetTrackRate(double raRate, double deRate) override;
        // virtual bool SetTrackEnabled(bool enabled) override;

        // Guiding
        virtual IPState GuideNorth(uint32_t ms) override;
        virtual IPState GuideSouth(uint32_t ms) override;
        virtual IPState GuideEast(uint32_t ms) override;
        virtual IPState GuideWest(uint32_t ms) override;
        // these all call these two functions
        IPState GuideNS(int32_t ms);
        IPState GuideWE(int32_t ms);

        LFAST::SLEW_MODE slewMode;

        bool unparkRequested;
        bool newConnectionFlag;
    private:
        void mountSim();
        bool getMountRaDec();
        bool isSlewComplete();

        // bool sendMountOKCommand(const char *command, const char *errorMessage, uint8_t timeout = 3);
        bool sendMountOKCommand(LFAST::MessageGenerator &cmdMsg, const char *errorMessage, uint8_t timeout = 3);
        void sendMountPassthroughCommand(LFAST::MessageGenerator &cmdMsg, const char *errorMessage, uint8_t timeout = 3);
        bool checkMountStatus(std::string parameter);
        bool startOpenLoopMotion(uint8_t motion, uint16_t rate);
        bool stopOpenLoopMotion();
        bool setMountTracking(bool enable, double raRate, double decRate);

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

        double currentDEC{0};
        double currentRA{90};
        double targetDEC{0};
        double targetRA{0};

        unsigned int DBG_SCOPE{0};

        // Jog Rate
        // ISwitch JogModeS[NUM_JOG_MODES];
        // ISwitchVectorProperty JogModeSP;
        // INDI::PropertySwitch JogModeSP{2};

        INumber JogRateN[LFAST::NUM_AXES];
        INumberVectorProperty JogRateNP;

        // Guide Rate
        INumber GuideRateN[2];
        INumberVectorProperty GuideRateNP;
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
        INumber TrackRateN[2];
        INumberVectorProperty TrackRateNP;
};
