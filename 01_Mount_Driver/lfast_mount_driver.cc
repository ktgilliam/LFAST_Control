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
#include "indipropertyswitch.h"

#include <libnova/sidereal_time.h>
#include <libnova/transform.h>

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>

#include "../00_Utils/lfast_comms.h"

// We declare an auto pointer to LFAST_Mount.
std::unique_ptr<LFAST_Mount> lfast_mount(new LFAST_Mount());

#define GOTO_RATE 5        /* slew rate, degrees/s */
#define SLEW_RATE 0.5      /* slew rate, degrees/s */
#define FINE_SLEW_RATE 0.1 /* slew rate, degrees/s */

#define GOTO_LIMIT 5.5 /* Move at GOTO_RATE until distance from target is GOTO_LIMIT degrees */
#define SLEW_LIMIT 1   /* Move at SLEW_LIMIT until distance from target is SLEW_LIMIT degrees */

/* Preset Slew Speeds */
#define SLEWMODES 9
const double slewspeeds[SLEWMODES] = {1.0, 2.0, 4.0, 8.0, 32.0, 64.0, 128.0, 256.0, 512.0};

// clang-format off
#define SCOPE_CAPABILITIES           \
            ( TELESCOPE_CAN_GOTO     \
            | TELESCOPE_CAN_PARK     \
            | TELESCOPE_CAN_ABORT    \
            | TELESCOPE_CAN_SYNC     \
            | TELESCOPE_HAS_LOCATION \
            )
// | TELESCOPE_HAS_TIME 
// | TELESCOPE_HAS_TRACK_MODE
// | TELESCOPE_HAS_TRACK_RATE
// | TELESCOPE_CAN_CONTROL_TRACK
// | TELESCOPE_HAS_PIER_SID
// clang-format on

int scopeCapabilities;

LFAST_Mount::LFAST_Mount()
{
    setVersion(0, 2);

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
    scopeCapabilities = SCOPE_CAPABILITIES;

    SetTelescopeCapability(scopeCapabilities, 9);

    setTelescopeConnection(CONNECTION_TCP);

    // The mount is initially in IDLE state.
    TrackState = SCOPE_IDLE;

    // Let init the pulse guiding properties
    initGuiderProperties(getDeviceName(), MOTION_TAB);

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

    // Set the driver interface to indicate that we can also do pulse guiding
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    /* initialize random seed: */
    srand(static_cast<uint32_t>(time(nullptr)));
    // initalise axis positions, for GEM pointing at pole, counterweight down
    // axisPrimary.setDegrees(90.0);
    // axisPrimary.TrackRate(Axis::SIDEREAL);
    // axisSecondary.setDegrees(90.0);
    unparkRequested = false;
    newConnectionFlag = false;
}

const char *LFAST_Mount::getDefaultName()
{
    return "LFAST_Mount";
}

bool LFAST_Mount::initProperties()
{
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    for (int i = 0; i < SlewRateSP.nsp - 1; i++)
    {
        sprintf(SlewRateSP.sp[i].label, "%.fx", slewspeeds[i]);
        SlewRateSP.sp[i].aux = (void *)&slewspeeds[i];
    }

    // Set 64x as default speed
    SlewRateSP.sp[5].s = ISS_ON;

    /* How fast do we jog compared to sidereal rate */
    IUFillNumber(&JogRateN[LFAST::RA_AXIS], "JOG_RATE_WE", "W/E Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX, JOG_RATE_STEP,
                 JOG_RATE_VALUE);
    IUFillNumber(&JogRateN[LFAST::DEC_AXIS], "JOG_RATE_NS", "N/S Rate (arcmin)", "%g", JOG_RATE_MIN, JOG_RATE_MAX,
                 JOG_RATE_STEP,
                 JOG_RATE_VALUE);

    IUFillNumberVector(&JogRateNP, JogRateN, LFAST::NUM_AXES, getDeviceName(), "JOG_RATE", "Jog", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);

    // IUFillNumberVector(&JogRateNP, JogRateN, 2, getDeviceName(), "JOG_RATE", "Jog Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);
    /* How fast do we guide compared to sidereal rate */
    IUFillNumber(&GuideRateN[LFAST::RA_AXIS], "GUIDE_RATE_WE", "W/E Rate", "%1.1f", 0.0, 1.0, 0.1, 0.5);
    IUFillNumber(&GuideRateN[LFAST::DEC_AXIS], "GUIDE_RATE_NS", "N/S Rate", "%1.1f", 0.0, 1.0, 0.1, 0.5);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);
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

    initGuiderProperties(getDeviceName(), MOTION_TAB);
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    // Other stuff
    addAuxControls();
    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();

    // Add alignment properties
    InitAlignmentProperties(this);

    addPollPeriodControl();

    double currentRA = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double currentDEC = LocationN[LOCATION_LATITUDE].value > 0 ? 90 : -90;
    targetRA = currentRA;
    targetDEC = currentDEC;

    setDefaultPollingPeriod(100);
    return true;
}

bool LFAST_Mount::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {

        defineProperty(&JogRateNP);
        defineProperty(&TrackRateNP);

        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);

        // Initial HA to 0 and currentDEC (+90 or -90)
        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(currentDEC);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            // Axis 1 is RA/AZ
            // Axis 2 is DEC/ALT
            SetAxis1Park(0);
            SetAxis2Park(currentDEC);
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(currentDEC);
        }
        SetParked(checkMountStatus("IsParked"));

        if (checkMountStatus("IsTracking"))
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

        defineProperty(&HomeSP);
        // requestLocation();
    }
    else
    {
        // deleteProperty(TrackModeSP.name);
        deleteProperty(TrackRateNP.name);
        deleteProperty(LocationNP.name);
        deleteProperty(JogRateNP.name);
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
        deleteProperty(HomeSP.name);
    }

    return true;
}

bool LFAST_Mount::Handshake()
{
    if (isSimulation())
        return true;

    int rc = 0, nbytes_written = 0;

    LFAST::MessageGenerator hsMsg("MountMessage");
    hsMsg.addArgument("Handshake", (unsigned int)0xDEAD);
    hsMsg.addArgument("time", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    // hsMsg.addArgument("latitude", LocationN[LOCATION_LATITUDE].value);
    // hsMsg.addArgument("longitude", LocationN[LOCATION_LONGITUDE].value);
    auto commandStr = hsMsg.getMessageStr();
    auto pCMD = commandStr.c_str();
    // pCMD[strlen(commandStr.c_str())] = '\0';

    LOGF_DEBUG("Writing handshake command (pCMD= %s)", pCMD);
    //     if ((rc = tty_write(PortFD, pCMD, std::strlen(commandStr.c_str())+1, &nbytes_written)) != TTY_OK)
    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing Handshake to Mount TCP server. Result: %d", rc);
        return false;
    }

    int nbytes_read = 0;
    char pRES[MAXRBUF] = {0};
    if ((rc = tty_read_section(PortFD, pRES, '\0', LFAST_MOUNT_HANDSHAKE_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        if (rc == -4)
            LOGF_ERROR("Timeout reading Handshake from Mount TCP server. [%s]", pRES);
        else
            LOGF_ERROR("Error reading Handshake from Mount TCP server. Result(%d): %d", nbytes_read, rc);
        return false;
    }

    LFAST::MessageParser rxMsg(pRES);
    if (!rxMsg.succeeded())
    {
        LOGF_ERROR("Error parsing received data <%s>", pRES);
        return false;
    }
    unsigned int handshakeReturnVal = 0;
    bool lookupValid = rxMsg.lookup<unsigned int>("Handshake", &handshakeReturnVal);
    if (!lookupValid)
    {
        LOGF_ERROR("Missing Handshake key in response: %s", pRES);
        return false;
    }
    if (handshakeReturnVal != 0xbeef)
    {
        LOGF_ERROR("Handshake key didn't match. String result: %s, key result: %u", pRES, handshakeReturnVal);
        return false;
    }
    newConnectionFlag = true;
    return true;
}

bool LFAST_Mount::getMountRaDec()
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;

    double mountAlt = 0., mountAz = 0.;
    LOG_DEBUG("Requesting Mount Ra/Dec");

    LFAST::MessageGenerator getAltAzMsg("MountMessage");
    getAltAzMsg.addArgument("RequestRaDec", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    auto commandStr = getAltAzMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    LOGF_DEBUG("\tCMD: %s", pCMD);

    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("getMountRaDec(): Error writing Ra/Dec request to mount Mount TCP server. Response: %d", rc);
        return false;
    }

    char pRES[MAXRBUF] = {0};
    if ((rc = tty_read_section(PortFD, pRES, '\0', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("getMountRaDec(): Error reading Ra/Dec request response from mount Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("\t\tRES: %s", pRES);

    LFAST::MessageParser rxMsg(pRES);
    if (!rxMsg.succeeded())
    {
        LOGF_ERROR("getMountRaDec(): Error parsing received data: %s", pRES);
        return false;
    }
    else
    {
        bool lookup1Valid = rxMsg.lookup<double>("RA", &currentRA);
        bool lookup2Valid = rxMsg.lookup<double>("DEC", &currentDEC);
        if (!(lookup1Valid && lookup2Valid))
        {
            LOGF_ERROR("Missing RA/DEC key in response: %s", pRES);
            return false;
        }
        return true;
    }

    LOGF_ERROR("Error reading coordinates. Result: %s", pRES);
    return false;
}

bool LFAST_Mount::ReadScopeStatus()
{
    static unsigned int cmdCounter = 0;
    const unsigned int maxTries = 10;

    if (isSimulation())
    {
        mountSim();
        return true;
    }
    printScopeMode();

    if (newConnectionFlag)
    {
        requestLocation();
        if (checkMountStatus("IsParked"))
            SyncParkStatus(true);
        else
            SyncParkStatus(false);
        newConnectionFlag = false;
    }

    switch (TrackState)
    {
    case SCOPE_SLEWING:
        LOG_DEBUG("\tReadScopeStatus(): CHECKING IF SLEWING");
        // Check if Scope is done slewing
        if (checkMountStatus("IsSlewComplete"))
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
        break;
    case SCOPE_PARKED:
        if (unparkRequested)
        {
            if (!checkMountStatus("IsParked"))
            {
                SetParked(false);
                unparkRequested = false;
                cmdCounter = 0;
            }
            else
            {
                cmdCounter++;
                if (cmdCounter >= maxTries)
                {
                    LOGF_ERROR("COULD NOT UNPARK SCOPE [%d]", cmdCounter);
                    unparkRequested = false;
                    cmdCounter = 0;
                }
            }
        }
        break;
    case SCOPE_PARKING:
        if (checkMountStatus("IsParked"))
        {
            SetParked(true);
        }
        break;
    case SCOPE_IDLE:
        break;
    }

    LOG_DEBUG("\tReadScopeStatus(): CHECKING Ra/Dec");
    if (!getMountRaDec())
        return false;

    char RAStr[64], DecStr[64];

    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
    return true;
}

bool LFAST_Mount::Goto(double r, double d)
{
    targetRA = r;
    targetDEC = d;

    // ###
    LFAST::MessageGenerator gotoCommandMsg("MountMessage");
    // gotoCommandMsg.addArgument("getTrackingStatus", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    gotoCommandMsg.addArgument("slewToRa", targetRA);
    gotoCommandMsg.addArgument("slewToDec", targetDEC);
    auto commandStr = gotoCommandMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    if (!sendMountOKCommand(gotoCommandMsg, "Slewing to target"))
        return false;

    TrackState = SCOPE_SLEWING;

    char RAStr[64], DecStr[64];
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
    return true;
}

bool LFAST_Mount::checkMountStatus(std::string parameter)
{
    LFAST::MessageGenerator mountParkStatusMsg("MountMessage");
    mountParkStatusMsg.addArgument(parameter, get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    auto commandStr = mountParkStatusMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    LOGF_DEBUG("\tCMD: %s", pCMD);

    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing %s to Mount TCP server. Result: %d", parameter.c_str(), rc);
        return false;
    }

    char pRES[MAXRBUF] = {0};
    if ((rc = tty_read_section(PortFD, pRES, '\0', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading %s from Mount TCP server. Result: %d", parameter.c_str(), rc);
        return false;
    }

    LOGF_DEBUG("\tRES: %s", pRES);
    LFAST::MessageParser rxMsg(pRES);
    if (!rxMsg.succeeded())
    {
        LOGF_ERROR("%s: Error parsing received data <%s>", parameter.c_str(), pRES);
        return false;
    }
    else
    {
        bool paramVal;
        bool lookupValid = rxMsg.lookup<bool>(parameter, &paramVal);
        if (!lookupValid)
        {
            LOGF_ERROR("Missing %s key in response: %s", parameter.c_str(), pRES);
            // tcflush(PortFD, TCIFLUSH);
            return false;
        }

        return (paramVal);
    }

    LOGF_ERROR("Error checking for %s. Invalid response: %s", parameter, pRES);
    return false;
}

bool LFAST_Mount::requestLocation()
{
    LOG_INFO("Requesting Local Coordinates from mount.");
    LFAST::MessageGenerator mountParkStatusMsg("MountMessage");
    mountParkStatusMsg.addArgument("RequestLatLonAlt", true);
    auto commandStr = mountParkStatusMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    LOGF_DEBUG("\tCMD: %s", pCMD);

    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing RequestLatLonAlt to Mount TCP server. Result: %d", rc);
        return false;
    }

    char pRES[MAXRBUF] = {0};
    if ((rc = tty_read_section(PortFD, pRES, '\0', LFAST_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        LOGF_ERROR("Error reading RequestLatLonAlt from Mount TCP server. Result: %d", rc);
        return false;
    }

    LOGF_DEBUG("\tRES: %s", pRES);
    LFAST::MessageParser rxMsg(pRES);
    if (!rxMsg.succeeded())
    {
        LOGF_ERROR("%s: Error parsing received data <RequestLatLonAlt>", pRES);
        return false;
    }
    else
    {
        double latitude, longitude, altitude;
        bool latitudeGood = rxMsg.lookup<double>("LAT", &latitude);
        bool longitudeGood = rxMsg.lookup<double>("LON", &longitude);
        bool altitudeGood = rxMsg.lookup<double>("ALT", &altitude);
        if (!(latitudeGood && longitudeGood && altitudeGood))
        {
            LOGF_ERROR("Failed to parse lat(%d)/lon(%d)/alt(%d): %s", latitudeGood, longitudeGood, altitudeGood, pRES);
            LocationNP.s = IPS_ALERT;
            return false;
        }
        return updateLocation(latitude, longitude, altitude);
    }

    LOGF_ERROR("Error checking for RequestLatLonAlt. Invalid response: %s", pRES);
    return false;
}

bool LFAST_Mount::updateLocation(double latitude, double longitude, double elevation)
{
    LocationNP.np[LOCATION_LATITUDE].value = latitude;
    LocationNP.np[LOCATION_LONGITUDE].value = longitude;
    LocationNP.np[LOCATION_ELEVATION].value = elevation;
    // LocationNP.s = IPS_OK;
    LocationNP.s = IPS_ALERT;

    IDSetNumber(&LocationNP, nullptr);
    saveConfig(false, "GEOGRAPHIC_COORD");
    // INDI::AlignmentSubsystem
    INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers::UpdateLocation(latitude, longitude, elevation);

    return true;
}

bool LFAST_Mount::Sync(double ra, double dec)
{
    currentRA = ra;
    currentDEC = dec;

    LFAST::MessageGenerator syncDataMessage("MountMessage");
    syncDataMessage.addArgument("syncRaPosn", currentRA);
    syncDataMessage.addArgument("syncDecPosn", currentDEC);
    if (!sendMountOKCommand(syncDataMessage, "Syncing to target"))
        return false;

    
    INDI::AlignmentSubsystem::AlignmentDatabaseEntry NewEntry;
    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension = ra;
    NewEntry.Declination = dec;
    NewEntry.TelescopeDirection = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
    NewEntry.PrivateDataSize = 0;

    LOG_INFO("Sync is successful.");
    EqNP.s = IPS_OK;
    NewRaDec(currentRA, currentDEC);

    return true;
}

bool LFAST_Mount::Park()
{
    double targetHA = GetAxis1Park();
    double targetRA = range24(get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value) - targetHA);
    double targetDEC = GetAxis2Park();

    LFAST::MessageGenerator mountParkCmdMsg("MountMessage");
    mountParkCmdMsg.addArgument("Park", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    // mountParkCmdMsg.addArgument("NoDisconnect", true);

    LOG_INFO("SENDING PARK COMMAND.");
    if (!sendMountOKCommand(mountParkCmdMsg, "Parking mount"))
    {
        LOG_DEBUG("\tSENDING PARK COMMAND FAILED!");
        return false;
    }

    TrackState = SCOPE_PARKING;
    LOG_INFO("Parking telescope in progress...");

    return true;
}

bool LFAST_Mount::UnPark()
{
    LFAST::MessageGenerator mountParkCmdMsg("MountMessage");
    mountParkCmdMsg.addArgument("Unpark", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));

    LOG_INFO("SENDING UNPARK COMMAND.");
    if (!sendMountOKCommand(mountParkCmdMsg, "Unparking mount"))
    {
        return false;
    }
    // Confirm we unparked
    // if (checkMountStatus("IsParked"))
    //     LOG_ERROR("Could not unpark for some reason.");
    // else
    //     SetParked(false);
    unparkRequested = true;
    return true;
}

bool LFAST_Mount::Abort()
{
    // char pCMD[MAXRBUF] = {0};
    LFAST::MessageGenerator abortCmdMsg("MountMessage");
    abortCmdMsg.addArgument("AbortSlew", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));

    LOG_INFO("Sending ABORT Slew Command");
    // sendMountPassthroughCommand(abortCmdMsg, "Sending Abort Command");
    if (!sendMountOKCommand(abortCmdMsg, "Sending Abort Command"))
    {
        return false;
    }

    return true;
}

bool LFAST_Mount::findHome()
{
    // char pCMD[MAXRBUF] = {0};
    LOG_INFO("Sending Home Command");
    LFAST::MessageGenerator homeCmdMsg("MountMessage");
    homeCmdMsg.addArgument("FindHome", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));

    LOG_INFO("Sending Home Command");
    if (!sendMountOKCommand(homeCmdMsg, "Sending Home Command", LFAST_HOMING_TIMEOUT))
    {
        return false;
    }

    return true;
}

bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    int motion = (dir == DIRECTION_NORTH) ? LFAST::NORTH : LFAST::SOUTH;
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
            LOGF_INFO("Moving toward %s.", (motion == LFAST::NORTH) ? "North" : "South");
        break;

    case MOTION_STOP:
        if (!isSimulation() && !stopOpenLoopMotion())
        {
            LOG_ERROR("Error stopping N/S motion.");
            return false;
        }
        else
            LOGF_INFO("Moving toward %s halted.",
                      (motion == LFAST::NORTH) ? "North" : "South");
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

    int motion = (dir == DIRECTION_WEST) ? LFAST::WEST : LFAST::EAST;
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
            LOGF_INFO("Moving toward %s.", (motion == LFAST::WEST) ? "West" : "East");
        break;

    case MOTION_STOP:
        if (!isSimulation() && !stopOpenLoopMotion())
        {
            LOG_ERROR("Error stopping W/E motion.");
            return false;
        }
        else
            LOGF_INFO("Movement toward %s halted.",
                      (motion == LFAST::WEST) ? "West" : "East");
        break;
    }

    return true;
}

bool LFAST_Mount::startOpenLoopMotion(uint8_t motion, uint16_t rate)
{
    char pCMD[MAXRBUF] = {0};

    snprintf(pCMD, MAXRBUF, "LFAST_Mount.DoCommand(9,'%d|%d');", motion, rate);
    // return sendMountOKCommand(pCMD, "Starting open loop motion");
    return false;
}

bool LFAST_Mount::stopOpenLoopMotion()
{
    char pCMD[MAXRBUF] = {0};

    strncpy(pCMD, "LFAST_Mount.DoCommand(10,'');", MAXRBUF);
    // return sendMountOKCommand(pCMD, "Stopping open loop motion");
    return false;
}

bool LFAST_Mount::updateTime(ln_date *utc, double utc_offset)
{
    INDI_UNUSED(utc);
    INDI_UNUSED(utc_offset);
    return true;
}

bool LFAST_Mount::SetCurrentPark()
{
    char pCMD[MAXRBUF] = {0};

    // strncpy(pCMD, "LFAST_Mount.SetParkPosition();", MAXRBUF);
    // if (!sendMountOKCommand(pCMD, "Setting Park Position"))

    LFAST::MessageGenerator setParkPosnMsg("MountMessage");
    setParkPosnMsg.addArgument("SetParkPosition", get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));

    LOG_INFO("Sending Set Park Position Message");

    return false;

    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double ha = get_local_hour_angle(lst, currentRA);

    SetAxis1Park(ha);
    SetAxis2Park(currentDEC);

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

bool LFAST_Mount::sendMountOKCommand(LFAST::MessageGenerator &cmdMsg, const char *errorMessage, uint8_t timeout)
{
    auto commandStr = cmdMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    LOGF_DEBUG("sendMountOKCommand pCMD: %s", pCMD);

    tcflush(PortFD, TCIOFLUSH);

    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing sendMountOKCommand to Mount TCP server. Result: $%d", rc);
        return false;
    }

    char pRES[MAXRBUF] = {0};
    if ((rc = tty_read_section(PortFD, pRES, '\0', timeout, &nbytes_read)) != TTY_OK)
    {
        if (rc == -4)
            LOGF_ERROR("Timeout reading sendMountOKCommand from Mount TCP server.", rc);
        else
            LOGF_ERROR("Error reading sendMountOKCommand from Mount TCP server. Result: %d", rc);
        return false;
    }
    // pRES[nbytes_read] = '\0';
    LOGF_DEBUG("sendMountOKCommand pRES: %s", pRES);

    tcflush(PortFD, TCIOFLUSH);

    LFAST::MessageParser rxMsg(pRES);
    if (!rxMsg.succeeded())
    {
        LOGF_ERROR("sendMountOKCommand: Error parsing received data <%s>", pRES);
        return false;
    }
    else
    {
        bool resultFlag = true;
        LOGF_DEBUG("\tChecking %d args in: %s.", cmdMsg.numArgs(), pCMD);
        for (unsigned ii = 0; ii < cmdMsg.numArgs(); ++ii)
        {
            auto keyStr = cmdMsg.getArgKey(ii);
            LOGF_DEBUG("\t(%s)", keyStr.c_str());
            std::string valStr = {0};
            auto lookupValid = rxMsg.lookup<std::string>(keyStr, &valStr);
            if (!lookupValid)
            {
                LOGF_INFO("\tsendMountOKCommand: Missing %s key in response: %s", keyStr.c_str(), pRES);
                return false;
            }
            LOGF_DEBUG("ARG CHECK: %s->%s", keyStr.c_str(), valStr.c_str());
            resultFlag &= valStr.compare("$OK^") == 0;
            LOGF_DEBUG("%s RESULT: %d", keyStr.c_str(), resultFlag);
        }
        return resultFlag;
    }
}

void LFAST_Mount::sendMountPassthroughCommand(LFAST::MessageGenerator &cmdMsg, const char *errorMessage, uint8_t timeout)
{
    cmdMsg.addArgument("NOREPLY", true);
    auto commandStr = cmdMsg.getMessageStr();
    auto pCMD = commandStr.c_str();

    LOGF_DEBUG("sendMountPassthroughCommand pCMD: %s", pCMD);

    tcflush(PortFD, TCIOFLUSH);

    int rc = 0, nbytes_written = 0, nbytes_read = 0;
    if ((rc = tty_write(PortFD, pCMD, std::strlen(pCMD), &nbytes_written)) != TTY_OK)
    {
        LOGF_ERROR("Error writing sendMountOKCommand to Mount TCP server. Result: $%d", rc);
        return;
    }
    // pRES[nbytes_read] = '\0';
    tcflush(PortFD, TCIOFLUSH);

    return;
}

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
    double dDec = GuideRateN[LFAST::DEC_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    LFAST::MessageGenerator guideNSDataMessage("MountMessage");
    guideNSDataMessage.addArgument("dRA", 0.0);
    guideNSDataMessage.addArgument("dDEC", dDec);
    if (!sendMountOKCommand(guideNSDataMessage, "issuing NS guide command"))
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
    double dRA = GuideRateN[LFAST::RA_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    LFAST::MessageGenerator guideNSDataMessage("MountMessage");
    guideNSDataMessage.addArgument("dRA", dRA);
    guideNSDataMessage.addArgument("dDEC", 0.0);
    if (!sendMountOKCommand(guideNSDataMessage, "issuing NS guide command"))
        return IPS_ALERT;

    m_WETimer.start(ms);

    return IPS_BUSY;
}

// bool LFAST_Mount::setMountTracking(bool enable, double raRate, double decRate)
// {
//     LFAST::MessageGenerator trackCmdMessage("MountMessage");
//     trackCmdMessage.addArgument("EnableTrack", enable);
//     trackCmdMessage.addArgument("raRate", raRate);
//     trackCmdMessage.addArgument("decRate", decRate);
// LOG_INFO("SET MOUNT TRACKING !!!!!!!!!!!!");
//     return sendMountOKCommand(trackCmdMessage, "Setting tracking rate");
// }

// bool LFAST_Mount::SetTrackRate(double raRate, double deRate)
// {
//     LOG_INFO("SET TRACK RATE !!!!!!!!!!!!");
//     return setMountTracking(true, raRate, deRate);
// }

// bool LFAST_Mount::SetTrackMode(uint8_t mode)
// {
//     bool isSidereal = (mode == TRACK_SIDEREAL);
//     double dRA = 0, dDE = 0;
// LOG_INFO("SET TRACK MODE !!!!!!!!!!!!");
//     switch (mode)
//     {
// #if TRACK_SOLAR_ENABLED
//     case TRACK_SOLAR:
//         dRA = TRACKRATE_SOLAR;
//         break;
// #endif
// #if TRACK_LUNAR_ENABLED
//     case TRACK_LUNAR:
//         dRA = TRACKRATE_LUNAR;
//         break;
// #endif
// #if TRACK_CUSTOM_ENABLED
//     case TRACK_CUSTOM:
//         dRA = TrackRateN[LFAST::RA_AXIS].value;
//         dDE = TrackRateN[LFAST::DEC_AXIS].value;
//         break;
// #endif
// #if TRACK_ALT_AZ_ENABLED
//     case TRACK_ALT_AZ:
// #warning ALT AZ TRACKING NOT YET IMPLEMENTED.
//         break;
// #endif
//     case TRACK_SIDEREAL:
//     // Intentional fall-through
//     default:
//         dRA = TRACKRATE_SIDEREAL;
//         break;
//     }

//     return setMountTracking(true, dRA, dDE);
// }

// bool LFAST_Mount::SetTrackEnabled(bool enabled)
// {
//     // On engaging track, we simply set the current track mode and it will take care of the rest including custom track rates.
//     if (enabled)
//         return SetTrackMode(IUFindOnSwitchIndex(&TrackModeSP));
//     else
//         // Otherwise, simply switch everything off
//         return setMountTracking(false, 0.0, 0.0);
// }

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

// // Account for the quadrant in declination
// double Celestron::trimDecAngle(double angle)
// {
//     angle = angle - 360 * floor(angle / 360);
//     if (angle < 0)
//         angle += 360.0;

//     if ((angle > 90.) && (angle <= 270.))
//         angle = 180. - angle;
//     else if ((angle > 270.) && (angle <= 360.))
//         angle = angle - 360.;

//     return angle;
// }
void LFAST_Mount::printScopeMode()
{
    // TelescopeStatus
    switch (TrackState)
    {
    case SCOPE_IDLE:
        LOG_DEBUG("# TrackState: SCOPE_IDLE");
        break;
    case SCOPE_SLEWING:
        LOG_DEBUG("# TrackState: SCOPE_SLEWING");
        break;
    case SCOPE_TRACKING:
        LOG_DEBUG("# TrackState: SCOPE_TRACKING");
        break;
    case SCOPE_PARKING:
        LOG_DEBUG("# TrackState: SCOPE_PARKING");
        break;
    case SCOPE_PARKED:
        LOG_DEBUG("# TrackState: SCOPE_PARKED");
        break;
    }
}

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
    }

    //  if we didn't process it, continue up the chain, let somebody else give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool LFAST_Mount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    LOGF_DEBUG("SWITCH PRESSED: %s\r\n", name);

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

        // if (!strcmp(ParkSP.name, name))
    }

    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool LFAST_Mount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessBlobProperties(this, name, sizes, blobsizes, blobs, formats, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool LFAST_Mount::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if(strcmp(dev,getDeviceName())==0)
    {
        // Process alignment properties
        ProcessTextProperties(this, name, texts, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

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

    if (fabs(targetRA - currentRA) * 15. >= GOTO_LIMIT)
        da_ra = GOTO_RATE * dt;
    else if (fabs(targetRA - currentRA) * 15. >= SLEW_LIMIT)
        da_ra = SLEW_RATE * dt;
    else
        da_ra = FINE_SLEW_RATE * dt;

    if (fabs(targetDEC - currentDEC) >= GOTO_LIMIT)
        da_dec = GOTO_RATE * dt;
    else if (fabs(targetDEC - currentDEC) >= SLEW_LIMIT)
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

        dx = targetRA - currentRA;

        // Take shortest path
        if (fabs(dx) > 12)
            dx *= -1;

        if (fabs(dx) <= da_ra)
        {
            currentRA = targetRA;
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

        dx = targetDEC - currentDEC;
        if (fabs(dx) <= da_dec)
        {
            currentDEC = targetDEC;
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