/*
   "LFAST Mount Driver"

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file lfast_mount_driver.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example lfast_mount_driver.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include "lfast_mount_driver.h"

#include <cmath>
#include <memory>

#include "config.h"
#include "indicom.h"
#include "libindi/connectionplugins/connectiontcp.h"
// #include "libindi/connectionplugins/connectionserial.h"

static std::unique_ptr<LFASTMount> lfastMount(new LFASTMount());

LFASTMount::LFASTMount()
{
    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
    this->setTelescopeConnection(CONNECTION_TCP);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool LFASTMount::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();

    // Add Debug control so end user can turn debugging/loggin on and off
    addDebugControl();

    // Enable simulation mode so that serial connection in INDI::Telescope does not try
    // to attempt to perform a physical connection to the serial port.
    setSimulation(true);

    // Set telescope capabilities. 0 is for the the number of slew rates that we support. We have none for this simple driver.
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT, 0);

    // Add debug/simulation/etc controls to the driver.
    addAuxControls();

    tcpConnection = new Connection::TCP(this);
    tcpConnection->setLANSearchEnabled(true);

    tcpConnection->setConnectionType(Connection::TCP::TYPE_UDP);
    tcpConnection->registerHandshake([&]()
                                     { return Handshake(); });
    registerConnection(tcpConnection);

    tcpConnection->Connect();
    return true;
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool LFASTMount::Handshake()
{
    // When communicating with a real mount, we check here if commands are receieved
    // and acknolowedged by the mount. For SimpleScope, we simply return true.
    if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
        // return true;
    }

    int rc = 0, nbytes_written = 0, nbytes_read = 0;

    char pCMD[MAXRBUF] = {0}, pRES[MAXRBUF] = {0};

   if(tty_read(tcpConnection->getPortFD(), pRES, '#', 10, &nbytes_read) != TTY_OK)
   {
    LOG_ERROR("HANDSHAKE FAILED.");
   }
   else
   {
    pRES[nbytes_read] = '\n';
    LOGF_INFO("RECEIVED: %s\n", pRES);
   }


    // TODO: Any initial communciation needed with our device; we have an active
    // connection with a valid file descriptor called PortFD. This file descriptor
    // can be used with the tty_* functions in indicom.h

    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *LFASTMount::getDefaultName()
{
    return "LFAST Mount Control";
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool LFASTMount::Goto(double ra, double dec)
{
    targetRA = ra;
    targetDEC = dec;
    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    // Inform client we are slewing to a new position
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool LFASTMount::Abort()
{
    return true;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool LFASTMount::ReadScopeStatus()
{
    static struct timeval ltv
    {
        0, 0
    };
    struct timeval tv
    {
        0, 0
    };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    int nlocked;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;

    // Calculate how much we moved since last time
    da_ra = SLEW_RATE * dt;
    da_dec = SLEW_RATE * dt;

    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
    case SCOPE_SLEWING:
        // Wait until we are "locked" into positon for both RA & DEC axis
        nlocked = 0;

        // Calculate diff in RA
        dx = targetRA - currentRA;

        // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
        if (fabs(dx) * 15. <= da_ra)
        {
            currentRA = targetRA;
            nlocked++;
        }
        // Otherwise, increase RA
        else if (dx > 0)
            currentRA += da_ra / 15.;
        // Otherwise, decrease RA
        else
            currentRA -= da_ra / 15.;

        // Calculate diff in DEC
        dy = targetDEC - currentDEC;

        // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
        if (fabs(dy) <= da_dec)
        {
            currentDEC = targetDEC;
            nlocked++;
        }
        // Otherwise, increase DEC
        else if (dy > 0)
            currentDEC += da_dec;
        // Otherwise, decrease DEC
        else
            currentDEC -= da_dec;

        // Let's check if we recahed position for both RA/DEC
        if (nlocked == 2)
        {
            // Let's set state to TRACKING
            TrackState = SCOPE_TRACKING;

            LOG_INFO("Telescope slew is complete. Tracking...");
        }
        break;

    default:
        break;
    }

    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
    return true;
}

void LFASTMount::TimerHit()
{
    if (!isConnected())
        return;

    LOG_INFO("timer hit");

    // If you don't call SetTimer, we'll never get called again, until we disconnect
    // and reconnect.
    SetTimer(POLLMS);
}