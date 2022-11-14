/*
   INDI Developers Manual
   Tutorial #7

   "Simple telescope simulator"

   We construct a most basic (and useless) device driver to illustrate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

#include "LFAST_Mount.h"

#include "indicom.h"
#include "alignment/DriverCommon.h"

#include <libnova/julian_day.h>
#include <memory>
#include <exception>

#include "slew_drive.h"
#include "../00_Utils/astro_math.h"

#define PRINT_DEBUG_STUFF 1

// #include "mountConfig.h"

// using namespace INDI::AlignmentSubsystem;

namespace ALIGNMENT = INDI::AlignmentSubsystem;
// clang-format off
#define SCOPE_CAPABILITIES ( TELESCOPE_CAN_GOTO     \
            | TELESCOPE_CAN_PARK    \
            | TELESCOPE_HAS_LOCATION \
            | TELESCOPE_CAN_ABORT   \
            | TELESCOPE_CAN_SYNC    \
            )
// | TELESCOPE_HAS_TIME 
// | TELESCOPE_HAS_TRACK_MODE
// | TELESCOPE_HAS_TRACK_RATE
// | TELESCOPE_CAN_CONTROL_TRACK
// | TELESCOPE_HAS_PIER_SID
// clang-format on

/* Preset Slew Speeds */
const double constexpr default_park_posn_az = 00.0;
const double constexpr default_park_posn_alt = 0.0;
const unsigned int defaultPollingPeriod = 100;

// We declare an auto pointer to LFAST_Mount.
std::unique_ptr<LFAST_Mount> lfast_mount(new LFAST_Mount());

// Axis labels:
const char azLabel[] = "Az. Axis";
const char altLabel[] = "Alt. Axis";

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
LFAST_Mount::LFAST_Mount() : DBG_SIMULATOR(INDI::Logger::getInstance().addDebugLevel("Simulator Verbose", "SIMULATOR"))
{
    // Set up the basic configuration for the mount
    setVersion(0, 3);
    setTelescopeConnection(CONNECTION_TCP);
    SetTelescopeCapability(SCOPE_CAPABILITIES, 4);

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    AltitudeAxis = new SlewDrive(altLabel);
    AzimuthAxis = new SlewDrive(azLabel);
    initializeTimers();

    // Set the driver interface to indicate that we can also do pulse guiding
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
    gotoPending = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::initializeTimers()
{
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
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
LFAST_Mount::~LFAST_Mount()
{
    delete AltitudeAxis;
    delete AzimuthAxis;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
const char *LFAST_Mount::getDefaultName()
{
    return (const char *)"LFAST Mount Control";
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::initProperties()
{
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    // Let's simulate it to be an F/10 8" telescope
    ScopeParametersN[0].value = 203;
    ScopeParametersN[1].value = 2000;
    ScopeParametersN[2].value = 203;
    ScopeParametersN[3].value = 2000;

    TrackState = SCOPE_IDLE;

    // Set up parking info
    SetParkDataType(PARK_AZ_ALT);
    if (InitPark())
    {
        // If loading parking data is successful, we just set the default parking values.
        // Axis 1 is ALT
        // Axis 2 is AZ
        SetAxis1ParkDefault(default_park_posn_az);
        SetAxis2ParkDefault(default_park_posn_alt);
    }
    else
    {
        // Otherwise, we set all parking data to default in case no parking data is found.
        SetAxis1Park(default_park_posn_az);
        SetAxis1ParkDefault(default_park_posn_az);
        SetAxis2Park(default_park_posn_alt);
        SetAxis2ParkDefault(default_park_posn_alt);
    }

    // How fast do we guide compared to sidereal rate
    GuideRateNP[AXIS_RA].fill("GUIDE_RATE_WE", "W/E Rate", "%.1f", 0, 1, 0.1, 0.5);
    GuideRateNP[AXIS_DE].fill("GUIDE_RATE_NS", "N/S Rate", "%.1f", 0, 1, 0.1, 0.5);
    GuideRateNP.fill(getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);

#if 0
    // Since we have 4 slew rates, let's fill them out
    SlewRateSP[SLEW_GUIDE].fill( "SLEW_GUIDE", "Guide", ISS_OFF);
    SlewRateSP[SLEW_CENTERING].fill( "SLEW_CENTERING", "Centering", ISS_OFF);
    SlewRateSP[SLEW_FIND].fill( "SLEW_FIND", "Find", ISS_OFF);
    SlewRateSP[SLEW_MAX].fill( "SLEW_MAX", "Max", ISS_ON);
    SlewRateSP.fill(getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);
#endif

    LOG_WARN("Initial position hardcoded to parking position");
    AltitudeAxis->syncPosition(default_park_posn_alt);
    AzimuthAxis->syncPosition(default_park_posn_az);

    addAuxControls();

    setDefaultPollingPeriod(defaultPollingPeriod);
    setCurrentPollingPeriod(defaultPollingPeriod);

    // Add alignment properties
    InitAlignmentProperties(this);

    // Create and update slew rates

    // for (int ii = 0; ii < MountSlewRateSP.size(); ii++)
    // {
    //     char label[10] = {0};
    //     sprintf(label, "%.fx", LFAST::slewspeeds[ii]);
    //     MountSlewRateSP[ii].fill(label, label, ISS_OFF);
    //     MountSlewRateSP[ii].aux = (void *)&LFAST::slewspeeds[ii];
    // }

    // // Set fastest default speed
    // MountSlewRateSP.fill(getDeviceName(), "MOUNT_SLEW_SPEED", "Fast Slew", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    // MountSlewRateSP[LFAST::DEFAULT_SLEW_IDX].s = ISS_ON;
    // SetSlewRate(LFAST::DEFAULT_SLEW_IDX);

    // NTP Server Address text field
    NtpServerTP[0].fill("NTP_SERVER_ADDR", "NTP Server", "0.pool.ntp.arizona.edu");
    NtpServerTP.fill(getDeviceName(), "NTP_SERVER_ADDR", "NTP Server", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);
    defineProperty(&NtpServerTP);

    AzAltCoordsNP[AXIS_AZ].fill("AZ_COORDINATE", "Az Posn [deg]", "%6.4f", 0, 360, 0.001, default_park_posn_az);
    AzAltCoordsNP[AXIS_ALT].fill("ALT_COORDINATE", "Alt Posn [deg]", "%6.4f", -90, 90, 0.001, default_park_posn_alt);
    AzAltCoordsNP[AXIS_AZ_VEL].fill("AZ_VEL_COORDINATE", "Az Rate [deg/s]", "%6.4f", 0, 10000, 0.0001, 0);
    AzAltCoordsNP[AXIS_ALT_VEL].fill("ALT_VEL_COORDINATE", "Alt Rate [deg/s]", "%6.4f", 0, 10000, 0.0001, 0);

    AzAltCoordsNP.fill(getDeviceName(), "ALT_AZ_COORDINATES", "Horizontal Coordinates", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // deleteProperty(SlewRateSP.name);
    // deleteProperty(LANSearchSP.name);
    // this->telescopeConnection

    // TrackStateTP[0].fill("SCOPE_STATE", "Mount State", "INIT");
    // TrackStateTP.fill(getDeviceName(), "SCOPE_STATE", "Mount State", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    TrackStateSP[SCOPE_IDLE].fill("STATE_IDLE", "IDLE", ISS_OFF);
    TrackStateSP[SCOPE_SLEWING].fill("STATE_SLEWING", "SLEWING", ISS_OFF);
    TrackStateSP[SCOPE_TRACKING].fill("STATE_TRACKING", "TRACKING", ISS_OFF);
    TrackStateSP[SCOPE_PARKING].fill("STATE_PARKING", "PARKING", ISS_OFF);
    TrackStateSP[SCOPE_PARKED].fill("STATE_PARKED", "PARKED", ISS_OFF);
    TrackStateSP[SCOPE_IDLE].fill("STATE_INIT", "IDLE", ISS_ON);
    TrackStateSP.fill(getDeviceName(), "SCOPE_STATE", "Mount State", MOTION_TAB, IP_RO, ISR_1OFMANY, 60, IPS_IDLE);

    // IUFillSwitch(&AxisOneStateS[FULL_STOP], "FULL_STOP", "FULL_STOP", ISS_OFF);
    // IUFillSwitch(&AxisOneStateS[SLEWING], "SLEWING", "SLEWING", ISS_OFF);
    // IUFillSwitch(&AxisOneStateS[SLEWING_TO], "SLEWING_TO", "SLEWING_TO", ISS_OFF);
    // IUFillSwitch(&AxisOneStateS[SLEWING_FORWARD], "SLEWING_FORWARD", "SLEWING_FORWARD", ISS_OFF);
    // IUFillSwitch(&AxisOneStateS[HIGH_SPEED], "HIGH_SPEED", "HIGH_SPEED", ISS_OFF);
    // IUFillSwitch(&AxisOneStateS[NOT_INITIALISED], "NOT_INITIALISED", "NOT_INITIALISED", ISS_ON);
    // IUFillSwitchVector(&AxisOneStateSP, AxisOneStateS, 6, getDeviceName(), "AXIS_ONE_STATE", "Axis one state",
    //                    DetailedMountInfoPage, IP_RO, ISR_NOFMANY, 60, IPS_IDLE);
    // TrackStateLP.fill(getDeviceName(), "SCOPE_STATE", "Mount State", MAIN_CONTROL_TAB, IPS_IDLE);

    // Force the alignment system to always be on
    getSwitch("ALIGNMENT_SUBSYSTEM_ACTIVE")->sp[0].s = ISS_ON;
    SetApproximateMountAlignmentFromMountType(ALTAZ);

    // LOG_WARN("Initial Park status hardcoded");
    // SetParked(true);

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updateProperties()
{
    LOG_INFO("LFAST_Mount::updateProperties");
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        // Delete this to redfine later (basically moving it to the bottom)
        deleteProperty(AbortSP.name);

        defineProperty(&AzAltCoordsNP);

        defineProperty(&AbortSP);

        defineProperty(&TrackStateSP);
        // defineProperty(&MountSlewRateSP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);
        // defineProperty(&AxisOneStateSP);
    }
    else
    {
        // deleteProperty(MountSlewRateSP.getName());
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.getName());
        deleteProperty(TrackStateSP.getName());
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Connect()
{
    SetTimer(getCurrentPollingPeriod());
    return INDI::Telescope::Connect();
    // return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Disconnect()
{
    return INDI::Telescope::Disconnect();
    // return true;
}

bool LFAST_Mount::Handshake()
{
    LOG_WARN("Handshake not implemented yet.");
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Goto(double ra, double dec)
{
    if (gotoPending)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, m_SkyCurrentRADE.rightascension, 2, 3600);
        fs_sexa(DecStr, m_SkyCurrentRADE.declination, 2, 3600);
        LOGF_DEBUG("Iterative GOTO RA %lf DEC %lf (Current Sky RA %s DE %s)", ra, dec, RAStr,
                   DecStr);
    }
    else
    {
        if (TrackState != SCOPE_IDLE)
        {
            Abort();
            return false;
        }
    }

    updateTrackingTarget(ra, dec);

    // Call the alignment subsystem to translate the celestial reference frame coordinate
    // into a telescope reference frame coordinate
    // ALIGNMENT::TelescopeDirectionVector TDVCommand;
    // INDI::IHorizontalCoordinates AltAzCommand{0, 0};
    INDI::IHorizontalCoordinates AltAzCommand = getTrackingTargetAltAzPosition();

    DEBUGF(DBG_SIMULATOR, "Goto - Scope reference frame target altitude %lf azimuth %lf", AltAzCommand.altitude,
           AltAzCommand.azimuth);

    // TODO: Try/Catch
    AltitudeAxis->updateTrackCommands(AltAzCommand.altitude);
    AzimuthAxis->updateTrackCommands(AltAzCommand.azimuth);

    TrackState = SCOPE_SLEWING;

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::updateTrackingTarget(double ra, double dec)
{
    DEBUGF(DBG_SIMULATOR, "Goto - Celestial reference frame target right ascension %lf(%lf) declination %lf",
           ra * 360.0 / 24.0, ra, dec);
    if (IUFindSwitch(&CoordSP, "TRACK")->s == ISS_ON || IUFindSwitch(&CoordSP, "SLEW")->s == ISS_ON)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, ra, 2, 3600);
        fs_sexa(DecStr, dec, 2, 3600);
        m_SkyTrackingTarget.rightascension = ra;
        m_SkyTrackingTarget.declination = dec;
        DEBUG(DBG_SIMULATOR, "Goto - tracking requested");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
INDI::IHorizontalCoordinates LFAST_Mount::getTrackingTargetAltAzPosition()
{
    double ra = m_SkyTrackingTarget.rightascension;
    double dec = m_SkyTrackingTarget.declination;
    ALIGNMENT::TelescopeDirectionVector TDVCommand;

    INDI::IHorizontalCoordinates horizCoords{0, 0};
    if (TransformCelestialToTelescope(ra, dec, 0.0, TDVCommand))
    {
        // The alignment subsystem has successfully transformed my coordinate
        AltitudeAzimuthFromTelescopeDirectionVector(TDVCommand, horizCoords);
    }
    else
    {
        // The alignment subsystem cannot transform the coordinate.
        // Try some simple rotations using the stored observatory position if any

        INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
        EquatorialCoordinates.rightascension = ra;
        EquatorialCoordinates.declination = dec;
        INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &horizCoords);
        TDVCommand = TelescopeDirectionVectorFromAltitudeAzimuth(horizCoords);
        switch (GetApproximateMountAlignment())
        {
        case ALIGNMENT::ZENITH:
            break;

        case ALIGNMENT::NORTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 minus
            // the (positive)observatory latitude. The vector itself is rotated anticlockwise
            TDVCommand.RotateAroundY(m_Location.latitude - 90.0);
            break;

        case ALIGNMENT::SOUTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 plus
            // the (negative)observatory latitude. The vector itself is rotated clockwise
            TDVCommand.RotateAroundY(m_Location.latitude + 90.0);
            break;
        }
        AltitudeAzimuthFromTelescopeDirectionVector(TDVCommand, horizCoords);
    }

    if ((horizCoords.altitude > 90.0) || (horizCoords.altitude < -90.0))
    {
        DEBUG(DBG_SIMULATOR, "Goto - Altitude out of range");
        // This should not happen
        // return false;
    }

    if ((horizCoords.azimuth > 360.0) || (horizCoords.azimuth < -360.0))
    {
        DEBUG(DBG_SIMULATOR, "Goto - Azimuth out of range");
        // This should not happen
        // return false;
    }

    if (horizCoords.azimuth < 0.0)
    {
        DEBUG(DBG_SIMULATOR, "Goto - Azimuth negative");
        horizCoords.azimuth = 360.0 + horizCoords.azimuth;
    }

    return horizCoords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::SetSlewRate(int index)
{
    // int currentIndex = IUFindOnSwitchIndex(&SlewRateSP);
    double raVal{0};
    double deVal{0};
    double slewRateTmp{0};
    double azVal{0};
    double altVal{0};

    double mult = 1;

    switch (index)
    {
    case SLEW_GUIDE:
        raVal = GuideRateNP[AXIS_RA].getValue();
        deVal = GuideRateNP[AXIS_DE].getValue();
        LOGF_INFO("Guide Rate Updated. RA: %6.4f, Dec: %6.4f", raVal, deVal);
        LOG_WARN("Guide rates not correctly transformed to az/el.");
        mult = 1.0 + ((raVal + deVal) * 0.5); // Temporary placeholder
        break;
    case SLEW_CENTERING:
        LOG_WARN("Slew centering not implemented");
        break;
    case SLEW_MAX:
        mult = LFAST::slewspeeds[LFAST::NUM_SLEW_SPEEDS - 1];
        break;
    case SLEW_FIND:
        mult = LFAST::slewspeeds[index];
        break;
    }

    slewRateTmp = mult * SIDEREAL_RATE_DPS;
    azVal = slewRateTmp;
    altVal = slewRateTmp;

    LOGF_INFO("Setting slew rate to %.3fx Sidereal (%6.4f deg/s).", mult, slewRateTmp);
    AzimuthAxis->updateSlewRate(azVal);
    AltitudeAxis->updateSlewRate(altVal);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
INDI::IHorizontalCoordinates LFAST_Mount::HorizontalRates_geocentric2(double ha, double dec, double lat)
{

    LOG_INFO("=================");
    LOGF_INFO("2HA: %6.4f", ha);
    LOGF_INFO("2DEC: %6.4f", dec);
    LOGF_INFO("2LAT: %6.4f", lat);
    // double ha_rad = hrs2rad(ha);
    // double cHA = rad2deg(std::cos(ha_rad));
    // double sHA = rad2deg(std::sin(ha_rad));

    double cHA = std::cos(deg2rad(ha));
    double sHA = std::sin(deg2rad(ha));

    double cDEC = std::cos(deg2rad(dec));
    double sDEC = std::sin(deg2rad(dec));
    double cLAT = std::cos(deg2rad(lat));
    double sLAT = std::sin(deg2rad(lat));

    //
    // Angular Positions
    //
    // Altitude Angle
    double altTerm1 = sDEC * sLAT;
    double altTerm2 = cDEC * cLAT * cHA;
    double sALT = altTerm1 + altTerm2;
    double cALT = std::sqrt(1 - sALT * sALT);

    // double alt = atan2d(sALT, cALT);
    double alt = rad2deg(std::asin(deg2rad(sALT)));
    LOGF_INFO("2ALT: %6.4f", alt);

    // Azimuth Angle
    double azY = -1.0 * sHA * cDEC;
    double azX = sDEC * cLAT - cDEC * sLAT * cHA;
    double azDen = (azY * azY) + (azX * azX);
    double cAZ = azX / azDen;
    double sAZ = azY / azDen;

    double az = rad2deg(std::atan2(deg2rad(sAZ), deg2rad(cAZ)));
    LOGF_INFO("2AZ: %6.4f", az);

    // double azAngle_deg = atan2d(azNum, azDen);
    // double cAZ = cosd(azAngle_deg);

    // Parallactic Angle
    double parY = sHA * cLAT;
    double parX = sLAT * cDEC - sDEC * cLAT * cHA;
    double parDen = (parY * parY) + (parX * parX);
    double cPAR = parX / parDen;
    double sPAR = parY / parDen;

    //
    // Angular Rates
    // Note: For now disregarding non-sidereal targets (proper motion terms are ignored).

    // Altitude Rate
    double altArg1 = cLAT * sAZ;
    // double altArg2 = (proper motion derivative of non-sidereal target);
    double altRate_dps = LFAST::SiderealRate_degpersec * altArg1;

    // Azimuth Rate
    double azArg1 = cDEC * cPAR / cALT;
    // double azArg2 = (proper motion derivative of non-sidereal target);
    double azRate_dps = LFAST::SiderealRate_degpersec * azArg1;

    // // Parallactic Rate
    // double parArg1 = -1.0 * cLAT * cAZ / cALT;
    // // double parArg2 = (proper motion derivative of non-sidereal target);
    // azRate_radpersec = LFAST::SiderealRate_degpersec * parArg1;
    INDI::IHorizontalCoordinates vHz{0, 0};
    vHz.altitude = altRate_dps;
    vHz.azimuth = azRate_dps * -1;
    return vHz;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
INDI::IHorizontalCoordinates LFAST_Mount::getTrackingTargetAltAzRates()
{
    double ra = m_SkyTrackingTarget.rightascension;
    double dec = m_SkyTrackingTarget.declination;
    ALIGNMENT::TelescopeDirectionVector TDVCommand;
    INDI::IHorizontalCoordinates altAzVel{0, 0};

    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double ha = lst - ra;
    // The alignment subsystem has successfully transformed my coordinate
    LOGF_INFO("ha: %6.4f", ha);
    // LOGF_INFO("dec: %6.4f", dec);
    // LOGF_INFO("lat1: %6.4f", LocationN[LOCATION_LATITUDE].value);
    // LOGF_INFO("lat2: %6.4f", m_Location.latitude);
    altAzVel = HorizontalRates_geocentric2(ha, dec, LocationN[LOCATION_LATITUDE].value);

    return altAzVel;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Abort()
{
    gotoPending = false;
    AltitudeAxis->abortSlew();
    AzimuthAxis->abortSlew();

    if (MovementNSSP.s == IPS_BUSY)
    {
        IUResetSwitch(&MovementNSSP);
        MovementNSSP.s = IPS_IDLE;
        IDSetSwitch(&MovementNSSP, nullptr);
    }

    if (MovementWESP.s == IPS_BUSY)
    {
        MovementWESP.s = IPS_IDLE;
        IUResetSwitch(&MovementWESP);
        IDSetSwitch(&MovementWESP, nullptr);
    }

    if (EqNP.s == IPS_BUSY)
    {
        EqNP.s = IPS_IDLE;
        IDSetNumber(&EqNP, nullptr);
    }

    TrackState = SCOPE_IDLE;

    if (GuideNSNP.s == IPS_BUSY || GuideWENP.s == IPS_BUSY)
    {
        GuideNSNP.s = GuideWENP.s = IPS_IDLE;
        GuideNSN[0].value = GuideNSN[1].value = 0.0;
        GuideWEN[0].value = GuideWEN[1].value = 0.0;

        IDMessage(getDeviceName(), "Guide aborted.");
        IDSetNumber(&GuideNSNP, nullptr);
        IDSetNumber(&GuideWENP, nullptr);

        return true;
    }

    AbortSP.s = IPS_OK;
    IUResetSwitch(&AbortSP);
    IDSetSwitch(&AbortSP, nullptr);
    LOG_INFO("Telescope aborted.");

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                            char *formats[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentBLOBProperties(this, name, sizes, blobsizes, blobs, formats, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentNumberProperties(this, name, values, names, n);
    }

    // Guiding Rate
    if (GuideRateNP.isNameMatch(name))
    {
        // for (int ii = 0; ii < n; ii++)
        // {
        //     LOGF_INFO("[%d] %6.4f", ii, values[ii]);
        // }
        IUUpdateNumber(&GuideRateNP, values, names, n);
        GuideRateNP.setState(IPS_OK);
        IDSetNumber(&GuideRateNP, nullptr);
        return true;
    }
    if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
    {
        processGuiderProperties(name, values, names, n);
        return true;
    }

    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    }
    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentTextProperties(this, name, texts, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    // double speed =
    //     (dir == DIRECTION_NORTH) ? -GetSlewRate() : GetSlewRate();
    double movement = (dir == DIRECTION_NORTH) ? -1 * command : command;
    const char *dirStr = (dir == DIRECTION_NORTH) ? "North" : "South";
    LOGF_INFO("MoveNS: %6.4f", movement);

    switch (command)
    {
    case MOTION_START:
        DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
        // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
        // AzimuthAxis->updateRateOffset(speed);
        // m_ManualMotionActive = true;
        LOG_ERROR("MoveNS Not Implemented");
        break;

    case MOTION_STOP:
        DEBUGF(DBG_SCOPE, "Stopping Slew %s", dirStr);
        AltitudeAxis->slowStop();
        AzimuthAxis->slowStop();
        break;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    // double speed =
    //     (dir == DIRECTION_WEST) ? -GetSlewRate() : GetSlewRate();
    double movement = (dir == DIRECTION_WEST) ? -1 * command : command;

    const char *dirStr = (dir == DIRECTION_WEST) ? "West" : "East";
    LOGF_INFO("MoveWE: %6.4f", movement);
    switch (command)
    {
    case MOTION_START:
        DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
        // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
        // AzimuthAxis->updateRateOffset(speed);
        // m_ManualMotionActive = true;
        LOG_ERROR("MoveWE Not Implemented");
        break;

    case MOTION_STOP:
        DEBUGF(DBG_SCOPE, "Stopping Slew %s", dirStr);
        AltitudeAxis->slowStop();
        AzimuthAxis->slowStop();
        break;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Sync(double ra, double dec)
{

    ALIGNMENT::AlignmentDatabaseEntry NewEntry;

    double azFb, altFb;
    azFb = AzimuthAxis->getPositionFeedback();
    altFb = AltitudeAxis->getPositionFeedback();
    m_MountAltAz.azimuth = azFb;
    m_MountAltAz.altitude = altFb;

    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension = ra;
    NewEntry.Declination = dec;
    NewEntry.TelescopeDirection = TelescopeDirectionVectorFromAltitudeAzimuth(m_MountAltAz);
    NewEntry.PrivateDataSize = 0;

    AltitudeAxis->syncPosition(altFb);
    AzimuthAxis->syncPosition(azFb);

    if (!CheckForDuplicateSyncPoint(NewEntry))
    {
        GetAlignmentDatabase().push_back(NewEntry);

        // Tell the client about size change
        UpdateSize();

        // Tell the math plugin to reinitialise
        Initialise(this);

        return true;
    }

    LOGF_INFO("SYNC'D RA/DEC: %6.4f, %6.4f", ra, dec);
    TraceThisTick = true;
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Park()
{
    LOG_DEBUG("LFAST_Mount::Park");
    // INDI::IHorizontalCoordinates AltAzParkingSpot{default_park_posn_az, default_park_posn_alt};
    // INDI::IEquatorialCoordinates EquatorialCoordinates;
    // INDI::HorizontalToEquatorial(&AltAzParkingSpot, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
    // char RAStr[64], DecStr[64];
    // fs_sexa(RAStr, EquatorialCoordinates.rightascension, 2, 3600);
    // fs_sexa(DecStr, EquatorialCoordinates.declination, 2, 3600);
    // LOGF_INFO("Parked RA: %s Parked DEC: %s", RAStr, DecStr);
    // gotoPending = true;
    AltitudeAxis->updateTrackCommands(default_park_posn_alt);
    AzimuthAxis->updateTrackCommands(default_park_posn_az);

    // NewRaDec(EquatorialCoordinates.rightascension, EquatorialCoordinates.declination);
    TrackState = SCOPE_PARKING;

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::UnPark()
{
    LOG_DEBUG("LFAST_Mount::UnPark");
    AltitudeAxis->enable();
    AzimuthAxis->enable();
    SetParked(false);
    TrackState = SCOPE_IDLE;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updateLocation(double latitude, double longitude, double elevation)
{
    LOG_DEBUG("LFAST_Mount::updateLocation");
    UpdateLocation(latitude, longitude, elevation);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ReadScopeStatus()
{
    LOG_DEBUG("LFAST_Mount::ReadScopeStatus");
    double azPosnFb, altPosnFb, azRateFb, altRateFb;
    try
    {
        azPosnFb = AzimuthAxis->getPositionFeedback();
        altPosnFb = AltitudeAxis->getPositionFeedback();
        azRateFb = AzimuthAxis->getVelocityFeedback();
        altRateFb = AltitudeAxis->getVelocityFeedback();
    }
    catch (const std::exception &e)
    {
        LOG_ERROR(e.what());
    }

    // Update the state switch
    int stateIndex = IUFindOnSwitchIndex(&TrackStateSP);
    TrackStateSP[stateIndex].s = ISS_OFF;
    TrackStateSP[TrackState].s = ISS_ON;
    IDSetSwitch(&TrackStateSP, nullptr);

    // Calculate new RA DEC
    if (updatePointingCoordinates(altPosnFb, azPosnFb))
    {
        NewRaDec(m_SkyCurrentRADE.rightascension, m_SkyCurrentRADE.declination);
        AzAltCoordsNP[AXIS_AZ].setValue(m_MountAltAz.azimuth);
        AzAltCoordsNP[AXIS_ALT].setValue(m_MountAltAz.altitude);
        AzAltCoordsNP[AXIS_AZ_VEL].setValue(azRateFb);
        AzAltCoordsNP[AXIS_ALT_VEL].setValue(altRateFb);
        AzAltCoordsNP.apply();
    }

    if (TrackState == SCOPE_SLEWING)
    {
        if ((AzimuthAxis->isStopped()) && (AltitudeAxis->isStopped()))
        {
            // If iterative GOTO was already engaged, stop it.
            if (gotoPending)
                gotoPending = false;
            // If not, then perform the iterative GOTO once more.
            else
            {
                gotoPending = true;
                return Goto(m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination);
            }

            if (ISS_ON == IUFindSwitch(&CoordSP, "TRACK")->s)
            {
                // Goto has finished start tracking
                TrackState = SCOPE_TRACKING;
                // resetTrackingTimers = true;
            }
            else
            {
                TrackState = SCOPE_IDLE;
            }
        }
    }
    // else if (TrackState == SCOPE_PARKING)
    // {
    //     if (!AzimuthAxis->isSlewComplete() && !AltitudeAxis->isSlewComplete())
    //     {
    //         AzimuthAxis->slowStop();
    //         AltitudeAxis->slowStop();
    //         // SetParked(true);
    //         // LOG_DEBUG("Scope Parking");
    //     }
    //     else
    //     {
    //         // LOG_DEBUG("Scope Parked");
    //         // TrackState = SCOPE_PARKED;
    //     }
    // }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updatePointingCoordinates(double alt, double az)
{
    LOG_DEBUG("LFAST_Mount::updatePointingCoordinates");
    bool successFlag = true;
    m_MountAltAz.altitude = alt;
    m_MountAltAz.azimuth = az;
    // LOGF_INFO("updateEquatorialCoordinates: ALT=%6.4f, AZ=%6.4f", alt, az)
    ALIGNMENT::TelescopeDirectionVector TDV = TelescopeDirectionVectorFromAltitudeAzimuth(m_MountAltAz);
    // LOGF_DEBUG("TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);

    double RightAscension, Declination;
    if (!TransformTelescopeToCelestial(TDV, RightAscension, Declination))
    {
        if (TraceThisTick)
            DEBUG(DBG_SIMULATOR, "ReadScopeStatus - TransformTelescopeToCelestial failed");

        ALIGNMENT::TelescopeDirectionVector RotatedTDV(TDV);

        switch (GetApproximateMountAlignment())
        {
        case ALIGNMENT::ZENITH:
            if (TraceThisTick)
                DEBUG(DBG_SIMULATOR, "ReadScopeStatus - ApproximateMountAlignment ZENITH");
            break;

        case ALIGNMENT::NORTH_CELESTIAL_POLE:
            if (TraceThisTick)
                DEBUG(DBG_SIMULATOR, "ReadScopeStatus - ApproximateMountAlignment NORTH_CELESTIAL_POLE");
            // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 minus
            // the (positive)observatory latitude. The vector itself is rotated clockwise
            RotatedTDV.RotateAroundY(90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, m_MountAltAz);
            break;

        case ALIGNMENT::SOUTH_CELESTIAL_POLE:
            if (TraceThisTick)
                DEBUG(DBG_SIMULATOR, "ReadScopeStatus - ApproximateMountAlignment SOUTH_CELESTIAL_POLE");
            // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 plus
            // the (negative)observatory latitude. The vector itself is rotated anticlockwise
            RotatedTDV.RotateAroundY(-90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, m_MountAltAz);
            break;
        }

        INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
        INDI::HorizontalToEquatorial(&m_MountAltAz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
        // libnova works in decimal degrees
        RightAscension = EquatorialCoordinates.rightascension;
        Declination = EquatorialCoordinates.declination;
    }
    if (TraceThisTick)
        // LOGF_INFO("ReadScopeStatus - RA %lf hours DEC %lf degrees", RightAscension, Declination);
        DEBUGF(DBG_SIMULATOR, "ReadScopeStatus - RA %lf hours DEC %lf degrees", RightAscension, Declination);

    m_SkyCurrentRADE.rightascension = RightAscension;
    m_SkyCurrentRADE.declination = Declination;

    return successFlag;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideNorth(uint32_t ms)
{
    return GuideNS(static_cast<int>(ms));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideSouth(uint32_t ms)
{
    return GuideNS(-static_cast<int>(ms));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideEast(uint32_t ms)
{
    return GuideWE(static_cast<int>(ms));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideWest(uint32_t ms)
{
    return GuideWE(static_cast<int>(ms));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideNS(int32_t ms)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }

    // Movement in arcseconds
    // Send async
    // double dDec = GuideRateN[LFAST::DEC_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    // LFAST::MessageGenerator guideNSDataMessage("MountMessage");
    // guideNSDataMessage.addArgument("dRA", 0.0);
    // guideNSDataMessage.addArgument("dDEC", dDec);

    // if (!sendMountOKCommand(guideNSDataMessage, "issuing NS guide command"))
    //     return IPS_ALERT;

    m_NSTimer.start(ms);

    return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideWE(int32_t ms)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }

    // Movement in arcseconds
    // Send async
    // double dRA = GuideRateN[LFAST::RA_AXIS].value * TRACKRATE_SIDEREAL * ms / 1000.0;
    // LFAST::MessageGenerator guideNSDataMessage("MountMessage");
    // guideNSDataMessage.addArgument("dRA", dRA);
    // guideNSDataMessage.addArgument("dDEC", 0.0);
    // if (!sendMountOKCommand(guideNSDataMessage, "issuing NS guide command"))
    //     return IPS_ALERT;

    m_WETimer.start(ms);

    return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double LFAST_Mount::GetSlewRate()
{
    LOG_DEBUG("LFAST_Mount::GetSlewRate");
    ISwitch *Switch = IUFindOnSwitch(&SlewRateSP);
    return *(static_cast<double *>(Switch->aux));
}

////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
// #include <vector>
void LFAST_Mount::printSlewDriveStates()
{
    static unsigned int count = 0;
    if (count++ >= 20)
    {
        double azPosnErr = AzimuthAxis->getPositionCommand() - AzimuthAxis->getPositionFeedback();
        double altPosnErr = AltitudeAxis->getPositionCommand() - AltitudeAxis->getPositionFeedback();
        LOGF_INFO("POSN ERR: %10.8f, %10.8f", azPosnErr, altPosnErr);

        // LOGF_INFO("POSN FB: %6.4f, %6.4f", AltitudeAxis->getPositionFeedback(), AzimuthAxis->getPositionFeedback());
        LOGF_INFO("RATE CMD: %10.8f, %10.8f", AltitudeAxis->getVelocityCommand(), AzimuthAxis->getVelocityCommand());
        LOGF_INFO("RATE FB: %10.8f, %10.8f", AltitudeAxis->getVelocityFeedback(), AzimuthAxis->getVelocityFeedback());
        count = 0;
    }
}

void LFAST_Mount::TimerHit()
{
    LOG_DEBUG("LFAST_Mount::TimerHit");
    TraceThisTickCount++;
    if (60 == TraceThisTickCount)
    {
        TraceThisTick = true;
        TraceThisTickCount = 0;
    }
    // Simulate mount movement

    updateSim();

    std::string stateStr = {0};
    INDI::IHorizontalCoordinates altAzPosn{0, 0};
    INDI::IHorizontalCoordinates altAzRates{0, 0};

    switch (TrackState)
    {
    case SCOPE_SLEWING:
        // TODO: RESET GUIDE PULSES
        // SEE skywatcherAPIMount
        if (AltitudeAxis->isSlewComplete() && AzimuthAxis->isSlewComplete())
        {
            LOG_INFO("SLEW COMPLETE.");
            TrackState = SCOPE_TRACKING;
        }
        break;

    case SCOPE_IDLE:
        break;
    case SCOPE_TRACKING:
        altAzPosn = getTrackingTargetAltAzPosition();
        // altAzRates = getTrackingTargetAltAzRates();

        AltitudeAxis->updateTrackCommands(altAzPosn.altitude, altAzRates.altitude);
        AzimuthAxis->updateTrackCommands(altAzPosn.azimuth, altAzRates.azimuth);
        // printSlewDriveStates();
        // LOGF_INFO("ALT/AZ Rate Commands: %10.8f, %10.8f",altAzRates.altitude, altAzRates.azimuth );
        break;
    case SCOPE_PARKING:
        if (!AzimuthAxis->isSlewComplete() && !AltitudeAxis->isSlewComplete())
        {
            AltitudeAxis->updateTrackCommands(default_park_posn_alt);
            AzimuthAxis->updateTrackCommands(default_park_posn_az);
            // SetParked(true);
            // LOG_DEBUG("Scope Parking");
        }
        else
        {
            AzimuthAxis->slowStop();
            AltitudeAxis->slowStop();
            // LOG_DEBUG("Scope Parked");
            TrackState = SCOPE_PARKED;
        }
        break;
    case SCOPE_PARKED:
        break;
    }

#if 0 // PRINT_DEBUG_STUFF
    auto itr = AltitudeAxis->debugStrings.begin();
    while (itr != AltitudeAxis->debugStrings.end())
    {
        auto s = *itr;
        LOG_INFO(s.c_str());
        AltitudeAxis->debugStrings.erase(itr);
    }

    itr = AzimuthAxis->debugStrings.begin();
    while (itr != AzimuthAxis->debugStrings.end())
    {
        auto s = *itr;
        LOG_INFO(s.c_str());
        AzimuthAxis->debugStrings.erase(itr);
    }
#endif
    INDI::Telescope::TimerHit();
    // // RA axis
    // long SlewSteps = dt * AxisSlewRateRA;
    // bool CompleteRevolution = SlewSteps >= MICROSTEPS_PER_REVOLUTION;
    // SlewSteps = SlewSteps % MICROSTEPS_PER_REVOLUTION; // Just in case ;-)

    // switch (AxisStatusRA)
    // {
    // case STOPPED:
    //     // Do nothing
    //     break;

    // case SLEWING:
    // {
    //     DEBUGF(DBG_SIMULATOR,
    //            "TimerHit Slewing - RA Current Encoder %ld SlewSteps %ld Direction %d Target %ld Status %d",
    //            CurrentEncoderMicrostepsRA, SlewSteps, AxisDirectionRA, GotoTargetMicrostepsRA, AxisStatusRA);

    //     // Update the encoder
    //     if (FORWARD == AxisDirectionRA)
    //         CurrentEncoderMicrostepsRA += SlewSteps;
    //     else
    //         CurrentEncoderMicrostepsRA -= SlewSteps;
    //     if (CurrentEncoderMicrostepsRA < 0)
    //         CurrentEncoderMicrostepsRA += MICROSTEPS_PER_REVOLUTION;
    //     else if (CurrentEncoderMicrostepsRA >= MICROSTEPS_PER_REVOLUTION)
    //         CurrentEncoderMicrostepsRA -= MICROSTEPS_PER_REVOLUTION;

    //     DEBUGF(DBG_SIMULATOR, "TimerHit Slewing - RA New Encoder %d New Status %d", CurrentEncoderMicrostepsRA,
    //            AxisStatusRA);
    //     break;
    // }

    // case SLEWING_TO:
    // {
    //     DEBUGF(DBG_SIMULATOR,
    //            "TimerHit SlewingTo - RA Current Encoder %ld SlewSteps %ld Direction %d Target %ld Status %d",
    //            CurrentEncoderMicrostepsRA, SlewSteps, AxisDirectionRA, GotoTargetMicrostepsRA, AxisStatusRA);

    //     long OldEncoder = CurrentEncoderMicrostepsRA;
    //     // Update the encoder
    //     if (FORWARD == AxisDirectionRA)
    //         CurrentEncoderMicrostepsRA += SlewSteps;
    //     else
    //         CurrentEncoderMicrostepsRA -= SlewSteps;
    //     if (CurrentEncoderMicrostepsRA < 0)
    //         CurrentEncoderMicrostepsRA += MICROSTEPS_PER_REVOLUTION;
    //     else if (CurrentEncoderMicrostepsRA >= MICROSTEPS_PER_REVOLUTION)
    //         CurrentEncoderMicrostepsRA -= MICROSTEPS_PER_REVOLUTION;

    //     if (CompleteRevolution)
    //     {
    //         // Must have found the target
    //         AxisStatusRA = STOPPED;
    //         CurrentEncoderMicrostepsRA = GotoTargetMicrostepsRA;
    //     }
    //     else
    //     {
    //         bool FoundTarget = false;
    //         if (FORWARD == AxisDirectionRA)
    //         {
    //             if (CurrentEncoderMicrostepsRA < OldEncoder)
    //             {
    //                 // Two ranges to search
    //                 if ((GotoTargetMicrostepsRA >= OldEncoder) &&
    //                     (GotoTargetMicrostepsRA <= MICROSTEPS_PER_REVOLUTION))
    //                     FoundTarget = true;
    //                 else if ((GotoTargetMicrostepsRA >= 0) &&
    //                          (GotoTargetMicrostepsRA <= CurrentEncoderMicrostepsRA))
    //                     FoundTarget = true;
    //             }
    //             else if ((GotoTargetMicrostepsRA >= OldEncoder) &&
    //                      (GotoTargetMicrostepsRA <= CurrentEncoderMicrostepsRA))
    //                 FoundTarget = true;
    //         }
    //         else
    //         {
    //             if (CurrentEncoderMicrostepsRA > OldEncoder)
    //             {
    //                 // Two ranges to search
    //                 if ((GotoTargetMicrostepsRA >= 0) && (GotoTargetMicrostepsRA <= OldEncoder))
    //                     FoundTarget = true;
    //                 else if ((GotoTargetMicrostepsRA >= CurrentEncoderMicrostepsRA) &&
    //                          (GotoTargetMicrostepsRA <= MICROSTEPS_PER_REVOLUTION))
    //                     FoundTarget = true;
    //             }
    //             else if ((GotoTargetMicrostepsRA >= CurrentEncoderMicrostepsRA) &&
    //                      (GotoTargetMicrostepsRA <= OldEncoder))
    //                 FoundTarget = true;
    //         }
    //         if (FoundTarget)
    //         {
    //             AxisStatusRA = STOPPED;
    //             CurrentEncoderMicrostepsRA = GotoTargetMicrostepsRA;
    //         }
    //     }
    //     DEBUGF(DBG_SIMULATOR, "TimerHit SlewingTo - RA New Encoder %d New Status %d", CurrentEncoderMicrostepsRA,
    //            AxisStatusRA);
    //     break;
    // }
    // }

    // // DEC axis
    // SlewSteps = dt * AxisSlewRateDEC;

    // switch (AxisStatusDEC)
    // {
    // case STOPPED:
    //     // Do nothing
    //     break;

    // case SLEWING:
    // {
    //     DEBUGF(DBG_SIMULATOR,
    //            "TimerHit Slewing - DEC Current Encoder %ld SlewSteps %d Direction %ld Target %ld Status %d",
    //            CurrentEncoderMicrostepsDEC, SlewSteps, AxisDirectionDEC, GotoTargetMicrostepsDEC, AxisStatusDEC);

    //     // Update the encoder
    //     SlewSteps = SlewSteps % MICROSTEPS_PER_REVOLUTION; // Just in case ;-)
    //     if (FORWARD == AxisDirectionDEC)
    //         CurrentEncoderMicrostepsDEC += SlewSteps;
    //     else
    //         CurrentEncoderMicrostepsDEC -= SlewSteps;
    //     if (CurrentEncoderMicrostepsDEC > MAX_DEC)
    //     {
    //         CurrentEncoderMicrostepsDEC = MAX_DEC;
    //         AxisStatusDEC = STOPPED; // Hit the buffers
    //         DEBUG(DBG_SIMULATOR, "TimerHit - DEC axis hit the buffers at MAX_DEC");
    //     }
    //     else if (CurrentEncoderMicrostepsDEC < MIN_DEC)
    //     {
    //         CurrentEncoderMicrostepsDEC = MIN_DEC;
    //         AxisStatusDEC = STOPPED; // Hit the buffers
    //         DEBUG(DBG_SIMULATOR, "TimerHit - DEC axis hit the buffers at MIN_DEC");
    //     }

    //     DEBUGF(DBG_SIMULATOR, "TimerHit Slewing - DEC New Encoder %d New Status %d", CurrentEncoderMicrostepsDEC,
    //            AxisStatusDEC);
    //     break;
    // }

    // case SLEWING_TO:
    // {
    //     DEBUGF(DBG_SIMULATOR,
    //            "TimerHit SlewingTo - DEC Current Encoder %ld SlewSteps %d Direction %ld Target %ld Status %d",
    //            CurrentEncoderMicrostepsDEC, SlewSteps, AxisDirectionDEC, GotoTargetMicrostepsDEC, AxisStatusDEC);

    //     // Calculate steps to target
    //     int StepsToTarget;
    //     if (FORWARD == AxisDirectionDEC)
    //     {
    //         if (CurrentEncoderMicrostepsDEC <= GotoTargetMicrostepsDEC)
    //             StepsToTarget = GotoTargetMicrostepsDEC - CurrentEncoderMicrostepsDEC;
    //         else
    //             StepsToTarget = CurrentEncoderMicrostepsDEC - GotoTargetMicrostepsDEC;
    //     }
    //     else
    //     {
    //         // Axis in reverse
    //         if (CurrentEncoderMicrostepsDEC >= GotoTargetMicrostepsDEC)
    //             StepsToTarget = CurrentEncoderMicrostepsDEC - GotoTargetMicrostepsDEC;
    //         else
    //             StepsToTarget = GotoTargetMicrostepsDEC - CurrentEncoderMicrostepsDEC;
    //     }
    //     if (StepsToTarget <= SlewSteps)
    //     {
    //         // Target was hit this tick
    //         AxisStatusDEC = STOPPED;
    //         CurrentEncoderMicrostepsDEC = GotoTargetMicrostepsDEC;
    //     }
    //     else
    //     {
    //         if (FORWARD == AxisDirectionDEC)
    //             CurrentEncoderMicrostepsDEC += SlewSteps;
    //         else
    //             CurrentEncoderMicrostepsDEC -= SlewSteps;
    //         if (CurrentEncoderMicrostepsDEC < 0)
    //             CurrentEncoderMicrostepsDEC += MICROSTEPS_PER_REVOLUTION;
    //         else if (CurrentEncoderMicrostepsDEC >= MICROSTEPS_PER_REVOLUTION)
    //             CurrentEncoderMicrostepsDEC -= MICROSTEPS_PER_REVOLUTION;
    //     }

    //     DEBUGF(DBG_SIMULATOR, "TimerHit SlewingTo - DEC New Encoder %d New Status %d", CurrentEncoderMicrostepsDEC,
    //            AxisStatusDEC);
    //     break;
    // }
    // }

    // INDI::Telescope::TimerHit(); // This will call ReadScopeStatus

    // // OK I have updated the celestial reference frame RA/DEC in ReadScopeStatus
    // // Now handle the tracking state
    // switch (TrackState)
    // {
    // case SCOPE_SLEWING:
    //     if ((STOPPED == AxisStatusRA) && (STOPPED == AxisStatusDEC))
    //     {
    //         if (ISS_ON == IUFindSwitch(&CoordSP, "TRACK")->s)
    //         {
    //             // Goto has finished start tracking
    //             DEBUG(DBG_SIMULATOR, "TimerHit - Goto finished start tracking");
    //             TrackState = SCOPE_TRACKING;
    //             // Fall through to tracking case
    //         }
    //         else
    //         {
    //             TrackState = SCOPE_IDLE;
    //             break;
    //         }
    //     }
    //     else
    //         break;

    // case SCOPE_TRACKING:
    // {
    //     // Continue or start tracking
    //     // Calculate where the mount needs to be in POLLMS time
    //     // POLLMS is hardcoded to be one second
    //     // TODO may need to make this longer to get a meaningful result
    //     double JulianOffset = 1.0 / (24.0 * 60 * 60);
    //     TelescopeDirectionVector TDV;
    //     INDI::IHorizontalCoordinates AltAz{0, 0};

    //     if (TransformCelestialToTelescope(m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination, JulianOffset,
    //                                       TDV))
    //         AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
    //     else
    //     {

    //         INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
    //         EquatorialCoordinates.rightascension = m_SkyTrackingTarget.rightascension;
    //         EquatorialCoordinates.declination = m_SkyTrackingTarget.declination;
    //         INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys() + JulianOffset, &AltAz);
    //         INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys() + JulianOffset,
    //                                      &AltAz);
    //     }

    //     // My altitude encoder runs -90 to +90
    //     if ((AltAz.altitude > 90.0) || (AltAz.altitude < -90.0))
    //     {
    //         DEBUG(DBG_SIMULATOR, "TimerHit tracking - Altitude out of range");
    //         // This should not happen
    //         return;
    //     }

    //     // My polar encoder runs 0 to +360
    //     if ((AltAz.azimuth > 360.0) || (AltAz.azimuth < -360.0))
    //     {
    //         DEBUG(DBG_SIMULATOR, "TimerHit tracking - Azimuth out of range");
    //         // This should not happen
    //         return;
    //     }

    //     if (AltAz.azimuth < 0.0)
    //     {
    //         DEBUG(DBG_SIMULATOR, "TimerHit tracking - Azimuth negative");
    //         AltAz.azimuth = 360.0 + AltAz.azimuth;
    //     }

    //     long AltitudeOffsetMicrosteps = int(AltAz.altitude * MICROSTEPS_PER_DEGREE - CurrentEncoderMicrostepsDEC);
    //     long AzimuthOffsetMicrosteps = int(AltAz.azimuth * MICROSTEPS_PER_DEGREE - CurrentEncoderMicrostepsRA);

    //     DEBUGF(DBG_SIMULATOR, "TimerHit - Tracking AltitudeOffsetMicrosteps %d AzimuthOffsetMicrosteps %d",
    //            AltitudeOffsetMicrosteps, AzimuthOffsetMicrosteps);

    //     if (0 != AzimuthOffsetMicrosteps)
    //     {
    //         // Calculate the slewing rates needed to reach that position
    //         // at the correct time. This is simple as interval is one second
    //         if (AzimuthOffsetMicrosteps > 0)
    //         {
    //             if (AzimuthOffsetMicrosteps < MICROSTEPS_PER_REVOLUTION / 2.0)
    //             {
    //                 // Forward
    //                 AxisDirectionRA = FORWARD;
    //                 AxisSlewRateRA = AzimuthOffsetMicrosteps;
    //             }
    //             else
    //             {
    //                 // Reverse
    //                 AxisDirectionRA = REVERSE;
    //                 AxisSlewRateRA = MICROSTEPS_PER_REVOLUTION - AzimuthOffsetMicrosteps;
    //             }
    //         }
    //         else
    //         {
    //             AzimuthOffsetMicrosteps = std::abs(AzimuthOffsetMicrosteps);
    //             if (AzimuthOffsetMicrosteps < MICROSTEPS_PER_REVOLUTION / 2.0)
    //             {
    //                 // Forward
    //                 AxisDirectionRA = REVERSE;
    //                 AxisSlewRateRA = AzimuthOffsetMicrosteps;
    //             }
    //             else
    //             {
    //                 // Reverse
    //                 AxisDirectionRA = FORWARD;
    //                 AxisSlewRateRA = MICROSTEPS_PER_REVOLUTION - AzimuthOffsetMicrosteps;
    //             }
    //         }
    //         AxisSlewRateRA = std::abs(AzimuthOffsetMicrosteps);
    //         AxisDirectionRA = AzimuthOffsetMicrosteps > 0 ? FORWARD : REVERSE; // !!!! BEWARE INERTIA FREE MOUNT
    //         AxisStatusRA = SLEWING;
    //         DEBUGF(DBG_SIMULATOR, "TimerHit - Tracking AxisSlewRateRA %lf AxisDirectionRA %d", AxisSlewRateRA,
    //                AxisDirectionRA);
    //     }
    //     else
    //     {
    //         // Nothing to do - stop the axis
    //         AxisStatusRA = STOPPED; // !!!! BEWARE INERTIA FREE MOUNT
    //         DEBUG(DBG_SIMULATOR, "TimerHit - Tracking nothing to do stopping RA axis");
    //     }

    //     if (0 != AltitudeOffsetMicrosteps)
    //     {
    //         // Calculate the slewing rates needed to reach that position
    //         // at the correct time.
    //         AxisSlewRateDEC = std::abs(AltitudeOffsetMicrosteps);
    //         AxisDirectionDEC = AltitudeOffsetMicrosteps > 0 ? FORWARD : REVERSE; // !!!! BEWARE INERTIA FREE MOUNT
    //         AxisStatusDEC = SLEWING;
    //         DEBUGF(DBG_SIMULATOR, "TimerHit - Tracking AxisSlewRateDEC %lf AxisDirectionDEC %d", AxisSlewRateDEC,
    //                AxisDirectionDEC);
    //     }
    //     else
    //     {
    //         // Nothing to do - stop the axis
    //         AxisStatusDEC = STOPPED; // !!!! BEWARE INERTIA FREE MOUNT
    //         DEBUG(DBG_SIMULATOR, "TimerHit - Tracking nothing to do stopping DEC axis");
    //     }

    //     break;
    // }

    // default:
    //     break;
    // }

    TraceThisTick = false;
}

void LFAST_Mount::hexDump(char *buf, const char *data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}

void LFAST_Mount::updateSim()
{
    LOG_DEBUG("LFAST_Mount::updateSim");
#if SIM_MODE_ENABLED
    static struct timeval ltv
    {
        0, 0
    }; // previous system time
    struct timeval tv
    {
        0, 0
    };         // new system time
    double dt; // Elapsed time in seconds since last tick

    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;
    // LOGF_INFO("dt: %6.4f", dt);
    AltitudeAxis->simulate(dt);
    AzimuthAxis->simulate(dt);
#endif
}