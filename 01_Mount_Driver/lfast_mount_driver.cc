/*******************************************************************************
 Copyright(c) 2017 Jasem Mutlaq. All rights reserved.
 2021 Chris Lewicki. Refactor of TCP connections and error handling

 Driver for using TheSkyX Pro Scripted operations for mounts via the TCP server.
 While this technically can operate any mount connected to the TheSkyX Pro, it is
 intended for LFAST_Mount mounts control.

 Ref TheSky Functions:
 https://www.bisque.com/wp-content/scripttheskyx/classsky6_r_a_s_c_o_m_tele.html

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

/*******************************************************************************
 * TODO for improving / completing the driver
 *
 * 1. SlewToAZAlt()
 * 2. GetAzAlt()
 * 3. (3) Presets for SlewToAZAlt positions
 * 4. DoCommand(16) get/set atmospheric pressure for interfacing with other INDI devices
 *******************************************************************************/

#include "lfast_mount_driver.h"

#include "indicom.h"

#include <libnova/sidereal_time.h>
#include <libnova/transform.h>

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>

// We declare an auto pointer to LFAST_Mount.
std::unique_ptr<LFAST_Mount> lfast_mount(new LFAST_Mount());

#define GOTO_RATE 5        /* slew rate, degrees/s */
#define SLEW_RATE 0.5      /* slew rate, degrees/s */
#define FINE_SLEW_RATE 0.1 /* slew rate, degrees/s */

#define GOTO_LIMIT 5.5 /* Move at GOTO_RATE until distance from target is GOTO_LIMIT degrees */
#define SLEW_LIMIT 1   /* Move at SLEW_LIMIT until distance from target is SLEW_LIMIT degrees */

#define LFAST_TIMEOUT 3 /* Timeout in seconds */

/* Preset Slew Speeds */
#define SLEWMODES 9
const double slewspeeds[SLEWMODES] = {1.0, 2.0, 4.0, 8.0, 32.0, 64.0, 128.0, 256.0, 512.0};
int scopeCapabilities;

std::string convertToString(double dblVal);
std::string getMessageIdString(int id);

LFAST_Mount::LFAST_Mount()
{
    setVersion(1, 4);

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
    scopeCapabilities = TELESCOPE_HAS_LOCATION
#if MOUNT_PARKING_ENABLED
                        | TELESCOPE_CAN_PARK
#endif
                        | TELESCOPE_CAN_ABORT | TELESCOPE_CAN_SYNC
                        // | TELESCOPE_HAS_TIME
                        | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_TRACK_MODE
        // | TELESCOPE_CAN_GOTO
        // | TELESCOPE_HAS_TRACK_RATE
        // | TELESCOPE_CAN_CONTROL_TRACK
        // | TELESCOPE_HAS_PIER_SIDE
        ;

    SetTelescopeCapability(scopeCapabilities, 9);

    setTelescopeConnection(CONNECTION_TCP);

#if MOUNT_GUIDER_ENABLED
    m_NSTimer.setSingleShot(true);
    m_WETimer.setSingleShot(true);
    // Called when timer is up
    m_NSTimer.callOnTimeout([this]()
                            {
        GuideNSNP.s = IPS_IDLE;
        GuideNSN[0].value = GuideNSN[1].value = 0;
        IDSetNumber(&GuideNSNP, nullptr); });

    m_WETimer.callOnTimeout([this]()
                            {
        GuideWENP.s = IPS_IDLE;
        GuideWEN[0].value = GuideWEN[1].value = 0;
        IDSetNumber(&GuideWENP, nullptr); });
#endif

    /* initialize random seed: */
    srand(static_cast<uint32_t>(time(nullptr)));
    // initalise axis positions, for GEM pointing at pole, counterweight down
    // axisPrimary.setDegrees(90.0);
    // axisPrimary.TrackRate(Axis::SIDEREAL);
    // axisSecondary.setDegrees(90.0);
}

const char *LFAST_Mount::getDefaultName()
{
    return "LFAST_Mount";
}

bool LFAST_Mount::initProperties()
{
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    // Delete properties we don't use
    // deleteProperty("USEJOYSTICK");

    for (int i = 0; i < SlewRateSP.nsp - 1; i++)
    {
        sprintf(SlewRateSP.sp[i].label, "%.fx", slewspeeds[i]);
        SlewRateSP.sp[i].aux = (void *)&slewspeeds[i];
    }

    // Set 64x as default speed
    SlewRateSP.sp[5].s = ISS_ON;

    /* How fast do we jog compared to sidereal rate */
    IUFillNumber(&JogRateN[RA_AXIS], "JOG_RATE_WE", "W/E Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX, JOG_RATE_STEP, JOG_RATE_VALUE);
    IUFillNumber(&JogRateN[DEC_AXIS], "JOG_RATE_NS", "N/S Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX, JOG_RATE_STEP, JOG_RATE_VALUE);
    IUFillNumber(&JogRateN[ALT_AXIS], "JOG_RATE_ALT", "Alt Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX, JOG_RATE_STEP, JOG_RATE_VALUE);
    IUFillNumber(&JogRateN[AZ_AXIS], "JOG_RATE_AZ", "Az Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX, JOG_RATE_STEP, JOG_RATE_VALUE);
    IUFillNumberVector(&JogRateNP, JogRateN, NUM_AXES, getDeviceName(), "JOG_RATE", "Jog Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);

    // IUFillNumberVector(&JogRateNP, JogRateN, 2, getDeviceName(), "JOG_RATE", "Jog Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);
#if MOUNT_GUIDER_ENABLED
    /* How fast do we guide compared to sidereal rate */
    IUFillNumber(&GuideRateN[RA_AXIS], "GUIDE_RATE_WE", "W/E Rate", "%1.1f", 0.0, 1.0, 0.1, 0.5);
    IUFillNumber(&GuideRateN[DEC_AXIS], "GUIDE_RATE_NS", "N/S Rate", "%1.1f", 0.0, 1.0, 0.1, 0.5);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);
#endif
    // Homing
    IUFillSwitch(&HomeS[0], "GO", "Go", ISS_OFF);
    IUFillSwitchVector(&HomeSP, HomeS, 1, getDeviceName(), "TELESCOPE_HOME", "Homing", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 60,
                       IPS_IDLE);
    // Tracking Mode
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
#if TRACK_SOLAR_ENABLED
    AddTrackMode("TRACK_SOLAR", "Solar");
#endif
#if TRACK_LUNAR_ENABLED
    AddTrackMode("TRACK_LUNAR", "Lunar");
#endif
#if TRACK_ALT_AZ_ENABLED
    AddTrackMode("TRACK_ALT_AZ", "Alt/Az");
#endif
#if TRACK_CUSTOM_ENABLED
    AddTrackMode("TRACK_CUSTOM", "Custom");
#endif
    // Let's simulate it to be an F/7.5 120mm telescope with 50m 175mm guide scope
    ScopeParametersN[0].value = 120;
    ScopeParametersN[1].value = 900;
    ScopeParametersN[2].value = 50;
    ScopeParametersN[3].value = 175;

    TrackState = SCOPE_IDLE;

    SetParkDataType(PARK_AZ_ALT);
#if MOUNT_GUIDER_ENABLED
    initGuiderProperties(getDeviceName(), MOTION_TAB);
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
#endif
    // Jogging
    IUFillSwitch(&JogModeS[JOG_MODE_RA_DEC], "JOG_MODE_RA_DEC", "Jog RA/DEC", ISS_ON);
    IUFillSwitch(&JogModeS[JOG_MODE_ALT_AZ], "JOG_MODE_ALT_AZ", "Jog ALT_AZ", ISS_OFF);
    IUFillSwitchVector(&JogModeSP, JogModeS, NUM_JOG_MODES, getDeviceName(), "JOG_MODE", "Jog Mode", MOTION_TAB, IP_RW, ISR_1OFMANY, 60,
                       IPS_IDLE);

    // Other stuff
    addAuxControls();
    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();

    double currentRA = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double currentDEC = LocationN[LOCATION_LATITUDE].value > 0 ? 90 : -90;
    setTargetRaDec(currentRA, currentDEC);

    setDefaultPollingPeriod(250);
    return true;
}

bool LFAST_Mount::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        if (isMountTracking())
        {
            IUResetSwitch(&TrackModeSP);
            TrackModeS[TRACK_SIDEREAL].s = ISS_ON;
            TrackState = SCOPE_TRACKING;
        }
        else
        {
            IUResetSwitch(&TrackModeSP);
            TrackState = SCOPE_IDLE;
        }

        defineProperty(&TrackModeSP);
        defineProperty(&TrackRateNP);

        defineProperty(&JogModeSP);
        defineProperty(&JogRateNP);

#if MOUNT_GUIDER_ENABLED
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);
#endif
#if MOUNT_PARKING_ENABLED
        // Initial HA to 0 and currentDEC (+90 or -90)
        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(currentALT);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(0);
            SetAxis2Park(currentALT);
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(currentALT);
        }
        SetParked(isMountParked());
#endif
        defineProperty(&HomeSP);
    }
    else
    {
        deleteProperty(TrackModeSP.name);
        deleteProperty(TrackRateNP.name);
        deleteProperty(JogModeSP.name);
        deleteProperty(JogRateNP.name);
#if MOUNT_GUIDER_ENABLED
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
#endif
        deleteProperty(HomeSP.name);
    }

    return true;
}

/*******************************************************************************
 * Note that for all successful TCP requests, the following string is
 * prepended to the result:
 *
 *    |Error=0.
 *
 * This is true everwhere except for the Handshake(), which just returns "1" on success.
 *
 * In order to know when the response is complete, we append the # character in
 * Javascript commands and read from the port until the # character is reached.
 *******************************************************************************/

bool LFAST_Mount::Handshake()
{
    if (isSimulation())
        return true;

    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

    strncpy(pCMD,
            "99#Handshake",
            MAXRBUF);

    LOGF_DEBUG("CMD: %s", pCMD);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing Handshake to Mount TCP server. Result: %d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading Handshake from Mount TCP server. Result: %d", rc);
        return false;
    }

    if (strcmp(pRES, "99#Handshake^") != 0)
    {
        LOGF_ERROR("Error connecting. Result: %s", pRES);
        return false;
    }
    else
    {
        LOGF_DEBUG("Got message ID: %s", pRES);
    }

    return true;
}

std::string convertToString(double dblVal)
{
    ByteConverter converter;
    converter.DOUBLE = dblVal;
    return (std::string((char *)converter.BYTES, 8));
}

std::string getMessageIdString(int id)
{
    // int2char i2c;
    // i2c.i = id;
    char idStr[] = {'#', '#', (char)id};
    return (std::string(idStr));
}

bool LFAST_Mount::getMountAltAz()
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};
    double mountAlt = 0., mountAz = 0.;
    LOG_DEBUG("Requesting Mount Alt/Az");

    strncpy(pCMD,
            "2#getAltAz",
            MAXRBUF);

    LOGF_DEBUG("CMD: %s", pCMD);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing GetRaDec to mount Mount TCP server. Response: %d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading GetRaDec from mount Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("RES: %s", pRES);

    // Read results successfully into temporary values before committing
    if (sscanf(pRES, "2#ALT=%lf;AZ=%lf^", &mountAlt, &mountAz) == 2)
    {
        currentALT = mountAlt;
        currentAZ = mountAz;
        return true;
    }

    LOGF_ERROR("Error reading coordinates. Result: %s", pRES);
    return false;
}

bool LFAST_Mount::ReadScopeStatus()
{
    if (isSimulation())
    {
        mountSim();
        return true;
    }

    if (TrackState == SCOPE_SLEWING)
    {
        // Check if Scope is done slewing
        if (isSlewComplete())
        {
            TrackState = SCOPE_TRACKING;

            if (HomeSP.s == IPS_BUSY)
            {
                IUResetSwitch(&HomeSP);
                HomeSP.s = IPS_OK;
                LOG_INFO("Finding home completed.");
            }
            else
                LOG_INFO("Slew is complete. Tracking...");
        }
    }
#if MOUNT_PARKING_ENABLED
    else if (TrackState == SCOPE_PARKING)
    {
        if (isMountParked())
        {
            SetParked(true);
        }
    }
#endif
    if (!getMountAltAz())
        return false;

    char RAStr[64], DecStr[64];

    double currentRA = this->getCurrentRa();
    double currentDEC = this->getCurrentDec();
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
    return true;
}

bool LFAST_Mount::Goto(double r, double d)
{
    double targetRA = r;
    double targetDEC = d;
    char RAStr[64], DecStr[64];

    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    char pCMD[MAXRBUF] = {0};
    snprintf(pCMD, MAXRBUF,
             "LFAST_Mount.Asynchronous = true;"
             "LFAST_Mount.SlewToRaDec(%g, %g,'');",
             targetRA, targetDEC);

    if (!sendTheSkyOKCommand(pCMD, "Slewing to target"))
        return false;

    TrackState = SCOPE_SLEWING;

    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
    return true;
}

bool LFAST_Mount::isSlewComplete()
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

    strncpy(pCMD,
            "4#IsSlewComplete",
            MAXRBUF);

    LOGF_DEBUG("CMD: %s", pCMD);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing IsSlewComplete to Mount TCP server. Result: %d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading IsSlewComplete from Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("RES: %s", pRES);

    int isComplete = 0;
    if (sscanf(pRES, "4#SlewIsComplete=%d^", &isComplete) == 1)
    {
        return isComplete == 1 ? 1 : 0;
    }

    LOGF_ERROR("Error reading isSlewComplete. Result: %s", pRES);
    return false;
}

#if MOUNT_PARKING_ENABLED
bool LFAST_Mount::isMountParked()
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

    strncpy(pCMD,
            "5#IsMountParked",
            MAXRBUF);

    LOGF_DEBUG("CMD: %s", pCMD);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing LFAST_Mount.IsParked() to Mount TCP server. Result: %d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading LFAST_Mount.IsParked() from Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("RES: %s", pRES);

    if (strcmp(pRES, "5#MountIsParked=true^") == 0)
        return true;
    if (strcmp(pRES, "5#MountIsParked=false^") == 0)
        return false;

    LOGF_ERROR("Error checking for park. Invalid response: %s", pRES);
    return false;
}
#endif

bool LFAST_Mount::isMountTracking()
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

    strncpy(pCMD,
            "3#getTrackingStatus",
            MAXRBUF);

    LOGF_DEBUG("CMD: %s", pCMD);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing LFAST_Mount.IsTracking to Mount TCP server. Result: %d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading LFAST_Mount.IsTracking from Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("RES: %s", pRES);

    double SkyXTrackRate = 0.;
    if (sscanf(pRES, "3#TrackRate=%lf^", &SkyXTrackRate) == 1)
    {
        if (SkyXTrackRate == 0)
            return false;
        else if (SkyXTrackRate > 0)
            return true;
    }

    LOGF_ERROR("Error checking for tracking. Invalid response: %s", pRES);
    return false;
}

bool LFAST_Mount::Sync(double ra, double dec)
{
    char pCMD[MAXRBUF] = {0};
    double alt, az;
    RaDecToAltAz(ra, dec, &alt, &az);

    snprintf(pCMD, MAXRBUF, "LFAST_Mount.Sync(%g, %g,'');", targetALT, targetAZ);
    if (!sendTheSkyOKCommand(pCMD, "Syncing to target"))
        return false;

    currentALT = alt;
    currentAZ = az;

    LOG_INFO("Sync is successful.");

    EqNP.s = IPS_OK;

    NewRaDec(ra, dec);

    return true;
}

#if MOUNT_PARKING_ENABLED
bool LFAST_Mount::Park()
{
    double targetHA = GetAxis1Park();
    double targetRA = range24(get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value) - targetHA);
    double targetDEC = GetAxis2Park();

    setTargetRaDec(targetRA, targetDEC);

    // char pCMD[MAXRBUF] = {0};
    // strncpy(pCMD,
    //         "LFAST_Mount.Asynchronous = true;"
    //         "LFAST_Mount.ParkAndDoNotDisconnect();",
    //         MAXRBUF);

    if (!sendTheSkyOKCommand("6#MountParkCommand", "Parking mount"))
        return false;
    TrackState = SCOPE_PARKING;
    LOG_INFO("Parking telescope in progress...");

    return true;
}

bool LFAST_Mount::UnPark()
{
    // char pCMD[MAXRBUF] = {0};
    // strncpy(pCMD, "LFAST_Mount.Unpark();", MAXRBUF);
    if (!sendTheSkyOKCommand("7#MountUnparkCommand", "Unparking mount"))
        return false;

    // Confirm we unparked
    if (isMountParked())
        LOG_ERROR("Could not unpark for some reason.");
    else
        SetParked(false);

    return true;
}
#endif

bool LFAST_Mount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, "JOG_RATE") == 0)
        {
            IUUpdateNumber(&JogRateNP, values, names, n);
            JogRateNP.s = IPS_OK;
            IDSetNumber(&JogRateNP, nullptr);
            return true;
        }
#if MOUNT_GUIDER_ENABLED
        // Guiding Rate
        if (strcmp(name, GuideRateNP.name) == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);
            return true;
        }
        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
#endif
    }

    //  if we didn't process it, continue up the chain, let somebody else give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool LFAST_Mount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(HomeSP.name, name))
        {
            LOG_INFO("Moving to home position. Please stand by...");
            if (findHome())
            {
                HomeS[0].s = ISS_OFF;
                TrackState = SCOPE_IDLE;
                HomeSP.s = IPS_OK;
                LOG_INFO("Mount arrived at home position.");
            }
            else
            {
                HomeS[0].s = ISS_OFF;
                HomeSP.s = IPS_ALERT;
                LOG_ERROR("Failed to go to home position");
            }

            IDSetSwitch(&HomeSP, nullptr);
            return true;
        }
    }

    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool LFAST_Mount::Abort()
{
    // char pCMD[MAXRBUF] = {0};
    LOG_INFO("Sending Abort Command");
    // strncpy(pCMD, "LFAST_Mount.Abort();", MAXRBUF);
    return sendTheSkyOKCommand("9#MountAbortCommand", "Abort mount slew");
}

bool LFAST_Mount::findHome()
{
    // char pCMD[MAXRBUF] = {0};
    LOG_INFO("Sending Home Command");
    // strncpy(pCMD, "LFAST_Mount.FindHome();"
    //               "while(!LFAST_Mount.IsSlewComplete) {"
    //               "sky6Web.Sleep(1000);}",
    //         MAXRBUF);
    return sendTheSkyOKCommand("8#MountHomeCommand", "Find home", 60);
}

bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    int motion = (dir == DIRECTION_NORTH) ? LFAST_NORTH : LFAST_SOUTH;
    // int rate   = IUFindOnSwitchIndex(&SlewRateSP);
    int rate = slewspeeds[IUFindOnSwitchIndex(&SlewRateSP)];

    switch (command)
    {
    case MOTION_START:
        if (!isSimulation() && !startOpenLoopMotion(motion, rate))
        {
            LOG_ERROR("Error setting N/S motion direction.");
            return false;
        }
        else
            LOGF_INFO("Moving toward %s.", (motion == LFAST_NORTH) ? "North" : "South");
        break;

    case MOTION_STOP:
        if (!isSimulation() && !stopOpenLoopMotion())
        {
            LOG_ERROR("Error stopping N/S motion.");
            return false;
        }
        else
            LOGF_INFO("Moving toward %s halted.",
                      (motion == LFAST_NORTH) ? "North" : "South");
        break;
    }

    return true;
}

bool LFAST_Mount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    int motion = (dir == DIRECTION_WEST) ? LFAST_WEST : LFAST_EAST;
    int rate = IUFindOnSwitchIndex(&SlewRateSP);

    switch (command)
    {
    case MOTION_START:
        if (!isSimulation() && !startOpenLoopMotion(motion, rate))
        {
            LOG_ERROR("Error setting W/E motion direction.");
            return false;
        }
        else
            LOGF_INFO("Moving toward %s.", (motion == LFAST_WEST) ? "West" : "East");
        break;

    case MOTION_STOP:
        if (!isSimulation() && !stopOpenLoopMotion())
        {
            LOG_ERROR("Error stopping W/E motion.");
            return false;
        }
        else
            LOGF_INFO("Movement toward %s halted.",
                      (motion == LFAST_WEST) ? "West" : "East");
        break;
    }

    return true;
}

bool LFAST_Mount::startOpenLoopMotion(uint8_t motion, uint16_t rate)
{
    char pCMD[MAXRBUF] = {0};

    snprintf(pCMD, MAXRBUF, "LFAST_Mount.DoCommand(9,'%d|%d');", motion, rate);
    return sendTheSkyOKCommand(pCMD, "Starting open loop motion");
}

bool LFAST_Mount::stopOpenLoopMotion()
{
    char pCMD[MAXRBUF] = {0};

    strncpy(pCMD, "LFAST_Mount.DoCommand(10,'');", MAXRBUF);
    return sendTheSkyOKCommand(pCMD, "Stopping open loop motion");
}

bool LFAST_Mount::updateTime(ln_date *utc, double utc_offset)
{
    INDI_UNUSED(utc);
    INDI_UNUSED(utc_offset);
    return true;
}

#if MOUNT_PARKING_ENABLED
bool LFAST_Mount::SetCurrentPark()
{
    char pCMD[MAXRBUF] = {0};

    strncpy(pCMD, "LFAST_Mount.SetParkPosition();", MAXRBUF);
    if (!sendTheSkyOKCommand(pCMD, "Setting Park Position"))
        return false;

    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double ha = get_local_hour_angle(lst, getCurrentRa());

    SetAxis1Park(currentAZ);
    SetAxis2Park(currentALT);

    return true;
}

bool LFAST_Mount::SetDefaultPark()
{
    // By default set Az to 0
    SetAxis1Park(0);

    // Set Alt to 90 or -90 depending on the hemisphere
    SetAxis2Park((LocationN[LOCATION_LATITUDE].value > 0) ? 90 : -90);

    return true;
}

bool LFAST_Mount::SetParkPosition(double Axis1Value, double Axis2Value)
{
    INDI_UNUSED(Axis1Value);
    INDI_UNUSED(Axis2Value);
    LOG_ERROR("Setting custom parking position directly is not supported. Slew to the desired "
              "parking position and click Current.");
    return false;
}
#endif

void LFAST_Mount::mountSim()
{
    static double currentRA, currentDEC;
    static bool firstTime = true;
    if (firstTime)
    {
        currentRA = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
        currentDEC = LocationN[LOCATION_LATITUDE].value > 0 ? 90 : -90;
        firstTime = false;
    }
    static struct timeval ltv
    {
        0, 0
    };
    struct timeval tv
    {
        0, 0
    };
    double dt, dx, da_ra = 0, da_dec = 0;
    int nlocked;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;

    if (fabs(getDeltaRa()) * 15. >= GOTO_LIMIT)
        da_ra = GOTO_RATE * dt;
    else if (fabs(getDeltaRa()) * 15. >= SLEW_LIMIT)
        da_ra = SLEW_RATE * dt;
    else
        da_ra = FINE_SLEW_RATE * dt;

    if (fabs(getDeltaDec()) >= GOTO_LIMIT)
        da_dec = GOTO_RATE * dt;
    else if (fabs(getDeltaDec()) >= SLEW_LIMIT)
        da_dec = SLEW_RATE * dt;
    else
        da_dec = FINE_SLEW_RATE * dt;

    double motionRate = 0;

    if (MovementNSSP.s == IPS_BUSY)
        motionRate = JogRateN[0].value;
    else if (MovementWESP.s == IPS_BUSY)
        motionRate = JogRateN[1].value;
    if (motionRate != 0)
    {
        da_ra = motionRate * dt * 0.05;
        da_dec = motionRate * dt * 0.05;

        switch (MovementNSSP.s)
        {
        case IPS_BUSY:
            if (MovementNSS[DIRECTION_NORTH].s == ISS_ON)
                currentDEC += da_dec;
            else if (MovementNSS[DIRECTION_SOUTH].s == ISS_ON)
                currentDEC -= da_dec;
            break;

        default:
            break;
        }

        switch (MovementWESP.s)
        {
        case IPS_BUSY:
            if (MovementWES[DIRECTION_WEST].s == ISS_ON)
                currentRA += da_ra / 15.;
            else if (MovementWES[DIRECTION_EAST].s == ISS_ON)
                currentRA -= da_ra / 15.;
            break;

        default:
            break;
        }

        NewRaDec(currentRA, currentDEC);
        return;
    }

    /* Process per current state. We check the state of EQUATORIAL_COORDS and act acoordingly */
    switch (TrackState)
    {
    case SCOPE_IDLE:
        /* RA moves at sidereal, Dec stands still */
        currentRA += (TRACKRATE_SIDEREAL / 3600.0 * dt / 15.);
        break;

    case SCOPE_SLEWING:
    case SCOPE_PARKING:
        /* slewing - nail it when both within one pulse @ SLEWRATE */
        nlocked = 0;

        dx = getDeltaRa();

        // Take shortest path
        if (fabs(dx) > 12)
            dx *= -1;

        if (fabs(dx) <= da_ra)
        {
            currentRA = getTargetRa();
            nlocked++;
        }
        else if (dx > 0)
            currentRA += da_ra / 15.;
        else
            currentRA -= da_ra / 15.;

        if (currentRA < 0)
            currentRA += 24;
        else if (currentRA > 24)
            currentRA -= 24;

        dx = getDeltaDec();
        if (fabs(dx) <= da_dec)
        {
            currentDEC = getTargetDec();
            nlocked++;
        }
        else if (dx > 0)
            currentDEC += da_dec;
        else
            currentDEC -= da_dec;

        if (nlocked == 2)
        {
            if (TrackState == SCOPE_SLEWING)
                TrackState = SCOPE_TRACKING;
            else
                SetParked(true);
        }
        break;

    default:
        break;
    }

    NewRaDec(currentRA, currentDEC);
}

bool LFAST_Mount::sendTheSkyOKCommand(const char *command, const char *errorMessage, uint8_t timeout)
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

    // snprintf(pCMD, MAXRBUF,
    //          "/* Java Script */"
    //          "var Out;"
    //          "try {"
    //          "%s"
    //          "Out  = 'OK#'; }"
    //          "catch (err) {Out = err; }",
    //          command);
    snprintf(pCMD, MAXRBUF, "%s", command);

    LOGF_DEBUG("CMD: %s", pCMD);

    tcflush(PortFD, TCIOFLUSH);

    if ((rc = tty_write_string(PortFD, pCMD, &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing sendTheSkyOKCommand to Mount TCP server. Result: $%d", rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, pRES, '^', timeout, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading sendTheSkyOKCommand from Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("RES: %s", pRES);

    tcflush(PortFD, TCIOFLUSH);

    char expectedResponse[MAXRBUF] = {0};
    snprintf(expectedResponse, MAXRBUF, "%s=$OK^", command);
    if (strcmp(expectedResponse, pRES) == 0)
        return true;
    else
    {
        LOGF_ERROR("sendTheSkyOKCommand Error %s - Invalid response: %s", errorMessage, pRES);
        return false;
    }
}
#if MOUNT_GUIDER_ENABLED
IPState LFAST_Mount::GuideNorth(uint32_t ms)
{
    return GuideNS(static_cast<int>(ms));
}

IPState LFAST_Mount::GuideSouth(uint32_t ms)
{
    return GuideNS(-static_cast<int>(ms));
}

IPState LFAST_Mount::GuideEast(uint32_t ms)
{
    return GuideWE(static_cast<int>(ms));
}

IPState LFAST_Mount::GuideWest(uint32_t ms)
{
    return GuideWE(-static_cast<int>(ms));
}

IPState LFAST_Mount::GuideNS(int32_t ms)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }

    // Movement in arcseconds
    // Send async
    double dDec = GuideRateN[DEC_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    char pCMD[MAXRBUF] = {0};
    snprintf(pCMD, MAXRBUF,
             "LFAST_Mount.Asynchronous = true;"
             "sky6DirectGuide.MoveTelescope(%g, %g);",
             0., dDec);

    if (!sendTheSkyOKCommand(pCMD, "Guide North-South"))
        return IPS_ALERT;

    m_NSTimer.start(ms);

    return IPS_BUSY;
}

IPState LFAST_Mount::GuideWE(int32_t ms)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }

    // Movement in arcseconds
    // Send async
    double dRA = GuideRateN[RA_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    char pCMD[MAXRBUF] = {0};
    snprintf(pCMD, MAXRBUF,
             "LFAST_Mount.Asynchronous = true;"
             "sky6DirectGuide.MoveTelescope(%g, %g);",
             dRA, 0.);

    if (!sendTheSkyOKCommand(pCMD, "Guide West-East"))
        return IPS_ALERT;

    m_WETimer.start(ms);

    return IPS_BUSY;
}
#endif
bool LFAST_Mount::setTheSkyTracking(bool enable, bool isSidereal, double raRate, double deRate)
{
    int on = enable ? 1 : 0;
    int ignore = isSidereal ? 1 : 0;
    char pCMD[MAXRBUF] = {0};

    snprintf(pCMD, MAXRBUF, "LFAST_Mount.SetTracking(%d, %d, %g, %g);", on, ignore, raRate, deRate);
    return sendTheSkyOKCommand(pCMD, "Setting tracking rate");
}

bool LFAST_Mount::SetTrackRate(double raRate, double deRate)
{
    return setTheSkyTracking(true, false, raRate, deRate);
}

bool LFAST_Mount::SetTrackMode(uint8_t mode)
{
    bool isSidereal = (mode == TRACK_SIDEREAL);
    double dRA = 0, dDE = 0;

    if (mode == TRACK_SOLAR)
        dRA = TRACKRATE_SOLAR;
    else if (mode == TRACK_LUNAR)
        dRA = TRACKRATE_LUNAR;
    else if (mode == TRACK_CUSTOM)
    {
        dRA = TrackRateN[RA_AXIS].value;
        dDE = TrackRateN[DEC_AXIS].value;
    }
    return setTheSkyTracking(true, isSidereal, dRA, dDE);
}

bool LFAST_Mount::SetTrackEnabled(bool enabled)
{
    // On engaging track, we simply set the current track mode and it will take care of the rest including custom track rates.
    if (enabled)
        return SetTrackMode(IUFindOnSwitchIndex(&TrackModeSP));
    else
        // Otherwise, simply switch everything off
        return setTheSkyTracking(false, false, 0.0, 0.0);
}

double LFAST_Mount::getCurrentRa()
{
    double currentRa;
    AltAzToRaDec(currentALT, currentAZ, &currentRa, 0);
    return currentRa;
}
double LFAST_Mount::getCurrentDec()
{
    double currentDec;
    AltAzToRaDec(currentALT, currentAZ, 0, &currentDec);
    return currentDec;
}
double LFAST_Mount::getTargetRa()
{
    double tgtRa;
    AltAzToRaDec(targetALT, targetAZ, &tgtRa, 0);
    return tgtRa;
}
double LFAST_Mount::getTargetDec()
{
    double tgtDec;
    AltAzToRaDec(targetALT, targetAZ, 0, &tgtDec);
    return tgtDec;
}
double LFAST_Mount::getDeltaRa()
{
    return (getTargetRa() - getCurrentRa());
}
double LFAST_Mount::getDeltaDec()
{
    return (getTargetDec() - getCurrentDec());
}

void LFAST_Mount::setTargetRaDec(double ra, double dec)
{
    double *tgtAz, *tgtAlt;
    tgtAz = &(this->targetAZ);
    tgtAlt = &(this->targetALT);
    RaDecToAltAz(ra, dec, tgtAlt, tgtAz);
}

void LFAST_Mount::AltAzToRaDec(double alt, double az, double *ra, double *dec)
{
    double haTmp;
    altAzToHADec(alt, az, &haTmp, dec);

    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    if (ra)
        *ra = lst - haTmp;
}

void LFAST_Mount::RaDecToAltAz(double ra, double dec, double *alt, double *az)
{
    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);

    double ha = (lst - ra);
    if (ha < 0)
    {
        ha += 2 * M_PI;
    }
    if (ha > M_PI)
    {
        ha = ha - 2 * M_PI;
    }
    double lat = LocationN[LOCATION_LATITUDE].value;
    double azTmp = atan2(sin(ha),
                         cos(ha) * sin(lat) - tan(dec) * cos(lat)) -
                   M_PI;
    if (az)
        *az = azTmp >= 0 ? azTmp : (azTmp + 2 * M_PI);

    double altTmp = asin(sin(lat) * sin(dec) + cos(lat) * cos(dec) * cos(ha));
    if (alt)
        *alt = altTmp;
}

void LFAST_Mount::altAzToHADec(double alt, double az, double *ha, double *dec)
{
    double lat = LocationN[LOCATION_LATITUDE].value;
    double haTmp = atan2(-sin(az), tan(alt) * cos(lat) - cos(az) * sin(lat));
    if (ha)
        *ha = haTmp >= 0 ? haTmp : haTmp + 2 * M_PI;

    double decTmp = asin(sin(lat) * sin(alt) + cos(lat) * cos(alt) * cos(az));
    if (dec)
        *dec = decTmp;
    return;
}