/*
   INDI Developers Manual
   Tutorial #7

   "Simple telescope simulator"

   We construct a most basic (and useless) device driver to illustrate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

#include "LFAST_Mount.h"
#include "config.h"

#include <libnova/julian_day.h>
#include <memory>
#include <exception>

#include "../00_Utils/math_util.h"
#include "slew_drive.h"
#include "lfast_constants.h"

// #define SIM_MODE 1
#define PRINT_DEBUG_STUFF 1

// http://docs.indilib.org/drivers/
// http://docs.indilib.org/drivers/project-setup.html
// http://docs.indilib.org/drivers/coming-from-ascom.html
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
/* | TELESCOPE_CAN_CONTROL_TRACK 
   | TELESCOPE_HAS_TIME 
   | TELESCOPE_HAS_TRACK_MODE
   | TELESCOPE_HAS_TRACK_RATE
   | TELESCOPE_HAS_PIER_SIDE */
// clang-format on

/* Preset Slew Speeds */
const double constexpr default_park_posn_az = 00.0;
const double constexpr default_park_posn_alt = -10.0;
const unsigned int defaultPollingPeriod_ms = 20;

// We declare an auto pointer to LFAST_Mount.
std::unique_ptr<LFAST_Mount> lfast_mount(new LFAST_Mount());

// Axis labels:
const char azLabel[] = "Azimuth";
const char altLabel[] = "Altitutde";

// const bool ALT_SIMULATED = SIM_MODE;
// const bool AZ_SIMULATED = SIM_MODE;

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
LFAST_Mount::LFAST_Mount()
    : TM_LOG(INDI::Logger::getInstance().addDebugLevel("Mount Telemetry", "TELEMETRY"))
{
    homingRoutineActive = false;
    // Set up the basic configuration for the mount
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    setTelescopeConnection(CONNECTION_TCP);
    SetTelescopeCapability(SCOPE_CAPABILITIES, LFAST_CONSTANTS::NUM_SLEW_SPEEDS);

    AzimuthAxis = std::unique_ptr<SlewDrive>(
        new SlewDrive(
            azLabel,
            LFAST_CONSTANTS::AZIMUTH_MOTOR_A_ID,
            LFAST_CONSTANTS::AZIMUTH_MOTOR_B_ID));

    AltitudeAxis = std::unique_ptr<SlewDrive>(
        new SlewDrive(
            altLabel,
            LFAST_CONSTANTS::ALTITUDE_MOTOR_A_ID,
            LFAST_CONSTANTS::ALTITUDE_MOTOR_B_ID));

    initializeTimers();

    // Set the driver interface to indicate that we can also do pulse guiding
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
    // gotoPending = false;
}

// void LFAST_Mount::logfTelemetry(const char* format, ...)
// {
//     char dest[1024 * 16];
//     va_list argptr;
//     va_start(argptr, format);
//     vsprintf(dest, format, argptr);
//     va_end(argptr);
//     DEBUGF(TM_LOG, "%s", dest);
// }
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::initializeTimers()
{
    m_NSTimer.setSingleShot(true);
    m_WETimer.setSingleShot(true);
    // Called when timer is up
    m_NSTimer.callOnTimeout([this]()
                            { terminateNSGuide(); });

    m_WETimer.callOnTimeout([this]()
                            { terminateEWGuide(); });
}

void LFAST_Mount::terminateNSGuide()
{
    guideManeuverActive = false;
    m_EqSkyGuideDelta.declination = 0;
    GuideNSNP.s = IPS_IDLE;
    GuideNSN[0].value = GuideNSN[1].value = 0;
    IDSetNumber(&GuideNSNP, nullptr);
    LOG_INFO("Terminated NS Guide");
}

void LFAST_Mount::terminateEWGuide()
{
    guideManeuverActive = false;
    m_EqSkyGuideDelta.rightascension = 0;
    GuideWENP.s = IPS_IDLE;
    GuideWEN[0].value = GuideWEN[1].value = 0;
    IDSetNumber(&GuideWENP, nullptr);
    LOG_INFO("Terminated EW Guide");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
LFAST_Mount::~LFAST_Mount()
{
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
    // https://indilib.org/develop/developer-manual/101-standard-properties.html
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    // Primary aperture and focal length
    ScopeParametersN[0].value = 762;
    ScopeParametersN[1].value = 2667;

    // Guider aperture and focal length
    // The celestron is an F/10 6" telescope
    ScopeParametersN[2].value = 152.40;
    ScopeParametersN[3].value = 1524;

    TrackState = SCOPE_IDLE;

    TelemetryDownsampleNP[0].fill("TELEMETRY_DOWNSAMPLE_NUM", "Telemetry Downsample", "%d", 1, 100, 1, DEFAULT_TM_TICKS_PER_UPDATE);
    TelemetryDownsampleNP.fill(getDeviceName(), "TELEMETRY_DOWNSAMPLE", "Telemetry Downsample", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    defineProperty(TelemetryDownsampleNP);


    // Set up parking info
    SetParkDataType(PARK_AZ_ALT);
    if (InitPark())
    {
        // If loading parking data is successful, we just set the default parking values.
        // alignment.latitude = Angle(LocationN[LOCATION_LATITUDE].value);
        // alignment.longitude = Angle(LocationN[LOCATION_LONGITUDE].value);
        // currentRA = (alignment.lst() - Angle(, Angle::ANGLE_UNITS::HOURS)).Hours();
        // currentDEC = ParkPositionN[AXIS_DE].value;
        LOG_INFO("Parking data found.");
        m_MountAltAz.altitude = ParkPositionN[AXIS_ALT].value;
        m_MountAltAz.azimuth = ParkPositionN[AXIS_AZ].value;
        INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
        INDI::HorizontalToEquatorial(&m_MountAltAz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
        Sync(EquatorialCoordinates.rightascension, EquatorialCoordinates.declination);
    }
    else
    {
        LOG_INFO("Parking data not found.");
        // Otherwise, we set all parking data to default in case no parking data is found.
        SetAxis1Park(default_park_posn_az);
        SetAxis1ParkDefault(default_park_posn_az);
        SetAxis2Park(default_park_posn_alt);
        SetAxis2ParkDefault(default_park_posn_alt);
        m_MountAltAz.altitude = default_park_posn_alt;
        m_MountAltAz.azimuth = default_park_posn_az;
    }

    initGuiderProperties(getDeviceName(), MOTION_TAB);

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

    // Create and update slew rates
    for (int i = 0; i < SlewRateSP.nsp; i++)
    {
        sprintf(SlewRateSP.sp[i].label, "%.fx", LFAST_CONSTANTS::slewspeeds[i]);
        SlewRateSP.sp[i].aux = (void *)&LFAST_CONSTANTS::slewspeeds[i];
        SlewRateSP.sp[i].s = ISS_OFF;
    }

    // Set default speed
    SlewRateSP.sp[LFAST_CONSTANTS::DEFAULT_SLEW_IDX].s = ISS_ON;

    addAuxControls();

    setDefaultPollingPeriod(defaultPollingPeriod_ms);
    setCurrentPollingPeriod(defaultPollingPeriod_ms);

    // Add alignment properties
    InitAlignmentProperties(this);

    // // Set fastest default speed
    // MountSlewRateSP.fill(getDeviceName(), "MOUNT_SLEW_SPEED", "Fast Slew", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    // MountSlewRateSP[LFAST::DEFAULT_SLEW_IDX].s = ISS_ON;
    SetSlewRate(LFAST_CONSTANTS::DEFAULT_SLEW_IDX);

    std::string modbusDevPath = "";
    for (int ii = 0; ii < 4; ii++)
    {
        char testDevPath[20];
        sprintf(testDevPath, "/dev/ttyUSB%d", ii);
        if (is_file_exist(testDevPath))
        {
            LOGF_INFO("FOUND: %s", testDevPath);
            modbusDevPath = testDevPath;
            break;
        }
    }

    // modbusDevPath = "/dev/ttyUSB0";
    ModbusCommPortTP[0].fill("MODBUS_COMM_DEV_TP", "Modbus Comm Port", modbusDevPath);
    ModbusCommPortTP.fill(getDeviceName(), "MODBUS_COMM_DEV", "Modbus", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);
    defineProperty(ModbusCommPortTP);

    AzAltCoordsNP[AXIS_AZ].fill("AZ_COORDINATE", "Az Posn [deg]", "%6.4f", 0, 360, 0.001, m_MountAltAz.azimuth);
    AzAltCoordsNP[AXIS_ALT].fill("ALT_COORDINATE", "Alt Posn [deg]", "%6.4f", -90, 90, 0.001, m_MountAltAz.altitude);
    AzAltCoordsNP[AXIS_AZ_VEL].fill("AZ_VEL_COORDINATE", "Az Rate [deg/s]", "%6.4f", 0, 10000, 0.0001, 0);
    AzAltCoordsNP[AXIS_ALT_VEL].fill("ALT_VEL_COORDINATE", "Alt Rate [deg/s]", "%6.4f", 0, 10000, 0.0001, 0);

    AzAltCoordsNP.fill(getDeviceName(), "ALT_AZ_COORDINATES", "Horizontal Coordinates", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // deleteProperty(SlewRateSP.name);
    // deleteProperty(LANSearchSP.name);
    // this->telescopeConnection

    // TrackStateTP[0].fill("SCOPE_STATE", "Mount State", "INIT");
    // TrackStateTP.fill(getDeviceName(), "SCOPE_STATE", "Mount State", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // TrackStateS[SCOPE_IDLE].fill("STATE_IDLE", "IDLE", ISS_OFF);
    // TrackStateS[SCOPE_SLEWING].fill("STATE_SLEWING", "SLEWING", ISS_OFF);
    // TrackStateS[SCOPE_TRACKING].fill("STATE_TRACKING", "TRACKING", ISS_OFF);
    // TrackStateSP[SCOPE_PARKING].fill("STATE_PARKING", "PARKING", ISS_OFF);
    // TrackStateSP[SCOPE_PARKED].fill("STATE_PARKED", "PARKED", ISS_OFF);
    // TrackStateSP[SCOPE_IDLE].fill("STATE_INIT", "IDLE", ISS_ON);
    // TrackStateSP.fill(getDeviceName(), "SCOPE_STATE", "Mount State", MAIN_CONTROL_TAB, IP_RO, ISR_1OFMANY, 60, IPS_IDLE);

    HomeSP[0].fill("GO_HOME", "Home", ISS_OFF);
    HomeSP.fill(getDeviceName(), "TELESCOPE_HOME", "Homing", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 300, IPS_IDLE);
    // Force the alignment system to always be on
    auto sw = getSwitch("ALIGNMENT_SUBSYSTEM_ACTIVE");

    // NOTE:
    // For some reason this needs to be ISS_ON on some computers, and IPS_OK on others.
    // If it won't compile, switch which line is commented out;
    // sw[0].s = ISS_ON;
    sw[0].s = IPS_OK;

    INDI::PropertySwitch
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
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        // Delete this to redfine later (basically moving it to the bottom)
        deleteProperty(AbortSP.name);

        // defineProperty(&TrackStateSP);
        defineProperty(HomeSP);
        defineProperty(AzAltCoordsNP);

        defineProperty(&AbortSP);

        // defineProperty(&MountSlewRateSP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(GuideRateNP);

        // defineProperty(&AxisOneStateSP);
    }
    else
    {
        // deleteProperty(MountSlewRateSP.getName());
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.getName());
        // deleteProperty(TrackStateSP.getName());
        deleteProperty(HomeSP.getName());
        deleteProperty(AzAltCoordsNP.getName());
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Connect()
{
    LOG_INFO("Connect()");
    SetTimer(getCurrentPollingPeriod());

    if (isSimulation())
    {
        AzimuthAxis->setSimulationMode(true);
        AltitudeAxis->setSimulationMode(true);
        AltitudeAxis->syncPosition(m_MountAltAz.altitude);
        AzimuthAxis->syncPosition(m_MountAltAz.azimuth);
        LOG_INFO("Fake connection established.");
        return true;
    }
    return INDI::Telescope::Connect();
    // return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Disconnect()
{
    LOG_INFO("Disconnect()");
    return INDI::Telescope::Disconnect();
    // return true;
}

bool LFAST_Mount::Handshake()
{
    LOG_INFO("Handshake()");
    // LOG_WARN("Handshake not implemented yet.");

    auto devPath = ModbusCommPortTP[0].getText();
    LOGF_INFO("Connecting to modbus comm port: %s...", devPath);
    try
    {
        SlewDrive::initializeDriverBus(devPath);
        AltitudeAxis->connectToDrivers();
        AzimuthAxis->connectToDrivers();
        // Zeros the encoders (probably will need to remove this!!)
        // AltitudeAxis->initializeStates();
        // AzimuthAxis->initializeStates();
        AltitudeAxis->syncPosition(m_MountAltAz.altitude);
        AzimuthAxis->syncPosition(m_MountAltAz.azimuth);
        LOG_INFO("Modbus connection established");
        return true;
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("HANDSHAKE FAILED: %s", e.what());
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Goto(double ra, double dec)
{
    bool success = true;

    LOGF_TM("Goto: [RA: %.6f], [DEC: %.6f]", ra, dec);

    updateTrackingTarget(ra, dec);

    INDI::IHorizontalCoordinates altAzTgtPosn{0, 0};
    altAzTgtPosn = getTrackingTargetAltAzPosition();
    AltitudeAxis->updateTrackCommands(altAzTgtPosn.altitude);
    AzimuthAxis->updateTrackCommands(altAzTgtPosn.azimuth);

    if (TrackState == SCOPE_IDLE)
    {
        try
        {
            AltitudeAxis->enable();
            AzimuthAxis->enable();
            TrackState = SCOPE_SLEWING;
            success = true;
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("Goto Error: %s", e.what());
            success = false;
        }
    }
    else if (TrackState == SCOPE_TRACKING)
    {
        TrackState = SCOPE_SLEWING;
    }
    return success;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::updateTrackingTarget(double ra, double dec)
{
    // DEBUGF(DBG_SIMULATOR, "Goto - Celestial reference frame target right ascension %lf(%lf) declination %lf",
    //    ra * 360.0 / 24.0, ra, dec);

    m_SkyGuideOffset = {0, 0};
    if (IUFindSwitch(&CoordSP, "TRACK")->s == ISS_ON || IUFindSwitch(&CoordSP, "SLEW")->s == ISS_ON)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, ra, 2, 3600);
        fs_sexa(DecStr, dec, 2, 3600);
        m_SkyTrackingTarget.rightascension = ra;
        m_SkyTrackingTarget.declination = dec;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
INDI::IHorizontalCoordinates LFAST_Mount::getTrackingTargetAltAzPosition()
{
    if (TraceThisTick)
        LOGF_TM("getTrackingTargetAltAzPosition: [Target RA: %.6f], [Target DEC: %.6f]", m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination);

    if (guideManeuverActive)
    {
        if (TraceThisTick)
            LOGF_TM("getTrackingTargetAltAzPosition: [Guide dRA: %.6f], [Guide dDEC: %.6f]", m_SkyGuideOffset.rightascension, m_SkyGuideOffset.declination);
        m_SkyGuideOffset.rightascension += m_EqSkyGuideDelta.rightascension;
        m_SkyGuideOffset.declination += m_EqSkyGuideDelta.declination;
    }
    double ra = m_SkyTrackingTarget.rightascension + m_SkyGuideOffset.rightascension;
    double dec = m_SkyTrackingTarget.declination + m_SkyGuideOffset.declination;
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
        LOG_WARN("Goto - Altitude out of range");
        // This should not happen
        // return false;
    }

    if ((horizCoords.azimuth > 360.0) || (horizCoords.azimuth < -360.0))
    {
        LOG_WARN("Goto - Azimuth out of range");
        // This should not happen
        // return false;
    }

    if (horizCoords.azimuth < 0.0)
    {
        LOG_INFO("Goto - Azimuth negative");
        horizCoords.azimuth = 360.0 + horizCoords.azimuth;
    }

    if (TraceThisTick)
        LOGF_TM("getTrackingTargetAltAzPosition: [Target ALT: %.6f], [Target AZ: %.6f]", horizCoords.altitude, horizCoords.azimuth);

    return horizCoords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::SetSlewRate(int index)
{
    double slewRateTmp{0};
    double azVal{0};
    double altVal{0};

    double mult = 1;

    mult = LFAST_CONSTANTS::slewspeeds[index];
    slewRateTmp = mult * LFAST_CONSTANTS::SiderealRate_degpersec;
    azVal = slewRateTmp;
    altVal = slewRateTmp;

    LOGF_TM("SetSlewRate: %.3fx Sidereal (%6.4f deg/s).", mult, slewRateTmp);

    AzimuthAxis->updateSlewRate(azVal);
    AltitudeAxis->updateSlewRate(altVal);
    return true;
}

// Calculate the feedforward rates for tracking whatever
// Currently only works for sidereal targets (nothing in the solar system)
// TODO: Update to accept an INDI::TelescopeTrackMode argument and calculate rates accordingly
INDI::IHorizontalCoordinates LFAST_Mount::getHorizontalRates()
{
    // Assumes sidereal target.
    double ra = m_SkyTrackingTarget.rightascension;
    double dec = m_SkyTrackingTarget.declination;

    double lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
    double ha = hrs2rad(lst - ra);
    // LOGF_DEBUG("HA: %6.5f", rad2deg(ha));

    // double cHA = std::cos(ha);
    double sHA = std::sin(ha);

    double cDEC = std::cos(deg2rad(dec));
    // double sDEC = std::sin(deg2rad(dec));

    double lat = deg2rad(LocationN[LOCATION_LATITUDE].value);
    double cLAT = std::cos(lat);
    // double sLAT = std::sin(lat);

    double alt = deg2rad(m_MountAltAz.altitude);
    // LOGF_DEBUG("ALT: %6.5f", rad2deg(alt));
    // double sALT = std::sin(alt);
    double cALT = std::cos(alt);

    double az = deg2rad(m_MountAltAz.azimuth);
    // LOGF_DEBUG("AZ: %6.5f", rad2deg(az));
    double sAZ = std::sin(az);
    // double cAZ = std::cos(az);

    double sPAR = sHA * cLAT / cALT;
    double PAR = std::asin(sPAR);
    // LOGF_DEBUG("PAR: %6.5f", rad2deg(PAR));
    double cPAR = std::cos(PAR);

    // double sum = PAR + ha + M_PI_2 - az;
    // LOGF_DEBUG("ANGLE SUM (deg): %6.4f", rad2deg(sum));
    // LOGF_DEBUG("ANGLE SUM (rad): %6.4f",(sum));

    double azRate_dps = LFAST_CONSTANTS::SiderealRate_degpersec * (cLAT * sAZ + 0.0);
    double altRate_dps = LFAST_CONSTANTS::SiderealRate_degpersec * (cDEC * cPAR / cALT + 0.0);

    INDI::IHorizontalCoordinates vHz{0, 0};
    vHz.altitude = altRate_dps;
    vHz.azimuth = azRate_dps;
    if (TraceThisTick)
        LOGF_TM("ALT_RATE: %6.4f, AZ_RATE: %6.4f", altRate_dps, azRate_dps);
    return vHz;
}
// //////////////////////////////////////////////////////////////////////////////////////////////////
// /// Tracking Rates V0
// //////////////////////////////////////////////////////////////////////////////////////////////////
INDI::IHorizontalCoordinates LFAST_Mount::getSiderealTargetAltAzRates()
{

    ALIGNMENT::TelescopeDirectionVector TDVCommand;
    INDI::IHorizontalCoordinates hzTrackRates{0, 0};

    hzTrackRates = getHorizontalRates();
    // LOGF_INFO("Rate Commands: dAlt = %6.4f, dAz = %6.4f", hzTrackRates.altitude, hzTrackRates.azimuth);

    return hzTrackRates;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Abort()
{
    // gotoPending = false;
    LOG_WARN("Abort()");
    AltitudeAxis->abortSlew();
    AzimuthAxis->abortSlew();
    m_SkyGuideOffset = {0, 0};
    m_EqSkyGuideDelta = {0, 0};

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
    if (homingRoutineActive)
    {
        AltitudeAxis->resetHomingRoutine();
        AzimuthAxis->resetHomingRoutine();
        HomeSP.setState(IPS_IDLE);
        // IDSetSwitch(&HomeSP, nullptr);
        HomeSP.apply();
        homingRoutineActive = false;
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
        AlignmentSubsystemForDrivers::ProcessAlignmentBLOBProperties(this, name, sizes, blobsizes, blobs, formats, names, n);
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
        // Guiding Rate
        if (GuideRateNP.isNameMatch(name))
        {
            GuideRateNP.update(values, names, n);
            GuideRateNP.setState(IPS_OK);
            GuideRateNP.apply();
            return true;
        }
        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
        if (TelemetryDownsampleNP.isNameMatch(name))
        {
            TelemetryDownsampleNP.update(values, names, n);
            TelemetryDownsampleNP.setState(IPS_OK);
            TelemetryDownsampleNP.apply();
            TraceThisTick = false;
            TraceThisTickCount = 0;
            return true;
        }
        // Process alignment properties
        AlignmentSubsystemForDrivers::ProcessAlignmentNumberProperties(this, name, values, names, n);
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
        if (HomeSP.isNameMatch(name))
        {
            HomeSP.setState(IPS_BUSY);
            // HomeSP.reset();
            HomeSP.apply();
            return startHomingRoutine();
        }
        // Process alignment properties
        AlignmentSubsystemForDrivers::ProcessAlignmentSwitchProperties(this, name, states, names, n);
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
        if (ModbusCommPortTP.isNameMatch(name))
        {
            ModbusCommPortTP.setState(IPS_OK);
            ModbusCommPortTP.update(texts, names, n);
            //  Update client display
            ModbusCommPortTP.apply();
            return true;
        }
        // Process alignment properties
        AlignmentSubsystemForDrivers::ProcessAlignmentTextProperties(this, name, texts, names, n);
    }

    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool LFAST_Mount::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    // Can't compile this at the moment.
    // Submitted a bug report to indi devs:
    // https://github.com/indilib/indi/issues/1947
    // IUSaveConfigNumber(fp, &TelemetryDownsampleNP);
    // IUSaveConfigText(fp, &ModbusCommPortTP);
    TelemetryDownsampleNP.save(fp);
    ModbusCommPortTP.save(fp);
    return true;
}

void LFAST_Mount::ISGetProperties(const char *dev)
{
    DefaultDevice::ISGetProperties(dev);

    // This would simulate a client sending a new value using the value stored in the config file.
    loadConfig(true, TelemetryDownsampleNP.getName());
    loadConfig(true, ModbusCommPortTP.getName());
}

void LFAST_Mount::simulationTriggered(bool enable)
{
    if (enable)
    {
        AltitudeAxis->setSimulationMode(true);
        AzimuthAxis->setSimulationMode(true);
        LOG_WARN("MOUNT IS IN SIMULATION MODE.");
    }
    else
    {
        bool connection_result = Handshake();
        if (connection_result)
        {
            AltitudeAxis->setSimulationMode(false);
            AzimuthAxis->setSimulationMode(false);
            // LOG_INFO("Succesfully exited simulation mode.");
            LOG_WARN("Warning: Exiting sim mode while connected may act weird. Recommend Powercycle.");
        }
        else
        {
            // Disconnect();
            LOG_ERROR("FAILED TO CONNECT - CANNOT DISABLE SIM MODE.");
            setSimulation(true);
        }
    }
}

bool LFAST_Mount::startHomingRoutine()
{
    LOG_ERROR("Homing Routine Not Implemented (Has a homing switch been installed yet?)");
    return false;
    // try
    // {
    //     if (TrackState == SCOPE_IDLE)
    //     {
    //         AltitudeAxis->enable();
    //         AzimuthAxis->enable();
    //     }

    //     homingRoutineActive = true;

    //     altHomingComplete = false;
    //     AltitudeAxis->startHoming();

    //     azHomingComplete = false;
    //     AzimuthAxis->startHoming();

    //     TrackState = SCOPE_SLEWING;
    //     LOG_INFO("Homing Routine Started.");
    //     return true;
    // }
    // catch (const std::exception &e)
    // {
    //     LOGF_ERROR("Homing Error: %s", e.what());
    //     return false;
    // }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (command == MOTION_STOP)
    {
        try
        {
            manualMotionActive = false;
            AltitudeAxis->updateManualRateCommand(0.0);
            AltitudeAxis->slowStop();
            TrackState = SCOPE_IDLE;
            LOG_INFO("Manual Slew Stopped.");
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("MoveNS Error: %s", e.what());
        }
    }
    else
    {
        double speedMult = (dir == DIRECTION_NORTH) ? GetSlewRate() : -1 * GetSlewRate();
        double speed = speedMult * LFAST_CONSTANTS::SiderealRate_degpersec;
        // if (TraceThisTick)
            LOGF_TM("MoveNS: %.6f", speed);

        switch (TrackState)
        {
        case SCOPE_IDLE:
            try
            {
                AltitudeAxis->enable();
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("MoveNS Error: %s", e.what());
            }
            // fall-through
        case SCOPE_TRACKING:
        case SCOPE_SLEWING:
        case SCOPE_PARKING:
            TrackState = SCOPE_SLEWING;
            manualMotionActive = true;
            AltitudeAxis->updateManualRateCommand(speed);
            break;
        case SCOPE_PARKED:
            LOG_WARN("WARNING: Cannot slew while scope is parked.");
        default:
            break;
        }
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (command == MOTION_STOP)
    {
        try
        {
            manualMotionActive = false;
            AzimuthAxis->updateManualRateCommand(0.0);
            AzimuthAxis->slowStop();
            LOG_INFO("Manual Slew Stopped.");
            TrackState = SCOPE_IDLE;
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("MoveWE Error: %s", e.what());
        }
    }
    else
    {
        double speedMult = (dir == DIRECTION_EAST) ? GetSlewRate() : -1 * GetSlewRate();
        double speed = speedMult * LFAST_CONSTANTS::SiderealRate_degpersec;
        // if (TraceThisTick)
            LOGF_TM("MoveWE: %.6f", speed);

        switch (TrackState)
        {
        case SCOPE_IDLE:
            try
            {
                AzimuthAxis->enable();
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("MoveNS Error: %s", e.what());
            }
            // fall-through
        case SCOPE_TRACKING:
        case SCOPE_SLEWING:
        case SCOPE_PARKING:
            TrackState = SCOPE_SLEWING;
            manualMotionActive = true;
            AzimuthAxis->updateManualRateCommand(speed);
            break;
        case SCOPE_PARKED:
            LOG_WARN("WARNING: Cannot slew while scope is parked.");
        default:
            break;
        }
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Sync(double ra, double dec)
{
    bool success = true;
    double azFb, altFb;
    LOGF_TM("Sync: [RA: %.6f], [DEC: %.6f]", ra, dec);
    if (!isSimulation())
    {
        try
        {
            azFb = AzimuthAxis->getPositionFeedback();
            altFb = AltitudeAxis->getPositionFeedback();
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("Sync Error: %s", e.what());
            success = false;
        }

        // TODO: This is bad do better:
        if (!success)
            return false;

        m_MountAltAz.azimuth = azFb;
        m_MountAltAz.altitude = altFb;
    }
    INDI::IHorizontalCoordinates newAltAz{0, 0};
    INDI::IEquatorialCoordinates syncRaDec{0, 0};
    syncRaDec.rightascension = ra;
    syncRaDec.declination = dec;
    INDI::EquatorialToHorizontal(&syncRaDec, &m_Location, ln_get_julian_from_sys(), &newAltAz);
    AltitudeAxis->syncPosition(newAltAz.altitude);
    AzimuthAxis->syncPosition(newAltAz.azimuth);
    LOGF_TM("Sync: [ALT: %.6f], [AZ: %.6f]", newAltAz.altitude, newAltAz.azimuth);
    
    ReadScopeStatus();
    // Syncing is treated specially when the telescope position is known in park position to spare
    // "a huge-jump point" in the alignment model.
    if (isParked())
    {
        GetAlignmentDatabase().clear();
        return false;
    }

    ALIGNMENT::AlignmentDatabaseEntry NewEntry;
    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension = ra;
    NewEntry.Declination = dec;
    NewEntry.TelescopeDirection = TelescopeDirectionVectorFromAltitudeAzimuth(m_MountAltAz);
    NewEntry.PrivateDataSize = 0;

    if (!CheckForDuplicateSyncPoint(NewEntry))
    {
        GetAlignmentDatabase().push_back(NewEntry);

        // Tell the client about size change
        UpdateSize();

        // Tell the math plugin to reinitialise
        Initialise(this);

        return true;
    }

    TraceThisTick = true;

    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Park()
{
    LOG_INFO("Park");
    // INDI::IHorizontalCoordinates AltAzParkingSpot{default_park_posn_az, default_park_posn_alt};
    // INDI::IEquatorialCoordinates EquatorialCoordinates;
    // INDI::HorizontalToEquatorial(&AltAzParkingSpot, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
    // char RAStr[64], DecStr[64];
    // fs_sexa(RAStr, EquatorialCoordinates.rightascension, 2, 3600);
    // fs_sexa(DecStr, EquatorialCoordinates.declination, 2, 3600);
    // LOGF_INFO("Parked RA: %s Parked DEC: %s", RAStr, DecStr);
    // gotoPending = true;
    if (TrackState != SCOPE_PARKED && TrackState != SCOPE_PARKING)
    {
        m_SkyGuideOffset = {0, 0};
        m_EqSkyGuideDelta = {0, 0};
        AltitudeAxis->updateTrackCommands(ParkPositionN[AXIS_ALT].value);
        AzimuthAxis->updateTrackCommands(ParkPositionN[AXIS_AZ].value);
        // NewRaDec(EquatorialCoordinates.rightascension, EquatorialCoordinates.declination);
        TrackState = SCOPE_PARKING;
    }

    return true;
}

bool LFAST_Mount::SetCurrentPark()
{
    SetAxis1Park(m_MountAltAz.azimuth);
    SetAxis2Park(m_MountAltAz.altitude);
    return true;
}

bool LFAST_Mount::SetDefaultPark()
{
    SetAxis1Park(default_park_posn_az);
    SetAxis2Park(default_park_posn_alt);
    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::UnPark()
{
    bool success = true;
    LOG_INFO("UnPark");
    if (TrackState != SCOPE_PARKING)
    {
        try
        {
            AltitudeAxis->enable();
            AzimuthAxis->enable();
            SetParked(false);
            TrackState = SCOPE_IDLE;
            success = true;
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("UnPark Error:%s", e.what());
            success = false;
        }
    }
    else
    {
        success = false;
        LOG_WARN("UNPARK WHILE PARKING");
    }
    return success;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updateLocation(double latitude, double longitude, double elevation)
{
    LOGF_TM("updateLocation: %.4f, %.4f, %.2f", latitude, longitude, elevation);
    UpdateLocation(latitude, longitude, elevation);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updatePointingCoordinates()
{
    bool successFlag = true;
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
        LOGF_ERROR("updatePointingCoordinates Error:%s", e.what());
        TrackState = SCOPE_IDLE;
        AzAltCoordsNP.setState(IPS_ALERT);
        AzAltCoordsNP.apply();
        successFlag = false;
    }
    if (!successFlag)
        return successFlag;

    m_MountAltAz.altitude = altPosnFb;
    m_MountAltAz.azimuth = azPosnFb;

    // LOGF_INFO("updateEquatorialCoordinates: ALT=%6.4f, AZ=%6.4f", alt, az)
    ALIGNMENT::TelescopeDirectionVector TDV = TelescopeDirectionVectorFromAltitudeAzimuth(m_MountAltAz);
    // LOGF_DEBUG("TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);

    double RightAscension, Declination;
    if (!TransformTelescopeToCelestial(TDV, RightAscension, Declination))
    {
        if (TraceThisTick)
            LOG_INFO("updatePointingCoordinates: TransformTelescopeToCelestial failed");

        ALIGNMENT::TelescopeDirectionVector RotatedTDV(TDV);

        switch (GetApproximateMountAlignment())
        {
        case ALIGNMENT::ZENITH:
            if (TraceThisTick)
                LOG_INFO("updatePointingCoordinates: ApproximateMountAlignment ZENITH");
            break;

        case ALIGNMENT::NORTH_CELESTIAL_POLE:
            if (TraceThisTick)
                LOG_INFO("updatePointingCoordinates: ApproximateMountAlignment NORTH_CELESTIAL_POLE");
            // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 minus
            // the (positive)observatory latitude. The vector itself is rotated clockwise
            RotatedTDV.RotateAroundY(90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, m_MountAltAz);
            break;

        case ALIGNMENT::SOUTH_CELESTIAL_POLE:
            if (TraceThisTick)
                LOG_INFO("updatePointingCoordinates: ApproximateMountAlignment SOUTH_CELESTIAL_POLE");
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
        LOGF_TM("ReadScopeStatus: RA %lf hours,  DEC %lf degrees", RightAscension, Declination);

    m_SkyCurrentRADE.rightascension = RightAscension;
    m_SkyCurrentRADE.declination = Declination;
    NewRaDec(m_SkyCurrentRADE.rightascension, m_SkyCurrentRADE.declination);

    AzAltCoordsNP[AXIS_AZ].setValue(m_MountAltAz.azimuth);
    AzAltCoordsNP[AXIS_ALT].setValue(m_MountAltAz.altitude);
    AzAltCoordsNP[AXIS_AZ_VEL].setValue(azRateFb);
    AzAltCoordsNP[AXIS_ALT_VEL].setValue(altRateFb);

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
IPState LFAST_Mount::GuideWest(uint32_t ms)
{
    return GuideWE(static_cast<int>(ms));
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideEast(uint32_t ms)
{
    return GuideWE(-static_cast<int>(ms));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideNS(int32_t guideDuration_ms)
{
    LOG_DEBUG("LFAST_Mount::GuideNS");
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }
    LOGF_TM("GuideNS: %d ms", guideDuration_ms);

    double numGuideSteps = abs(guideDuration_ms) / (double)defaultPollingPeriod_ms;
    double guideRate = GuideRateNP[AXIS_DE].getValue() * LFAST_CONSTANTS::SiderealRate_degpersec;
    double guideManeuverTotalDistanceDeg = guideRate * guideDuration_ms / 1000.0;
    double guideStepSizeDeg = guideManeuverTotalDistanceDeg / numGuideSteps;
    m_EqSkyGuideDelta.declination = guideStepSizeDeg;
    guideManeuverActive = true;
    m_NSTimer.start(abs(guideDuration_ms));

    return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideWE(int32_t guideDuration_ms)
{
    LOGF_INFO("LFAST_Mount::GuideWE: %d", guideDuration_ms);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return IPS_ALERT;
    }
    LOGF_TM("GuideWE: %d ms", guideDuration_ms);

    double numGuideSteps = abs(guideDuration_ms) / defaultPollingPeriod_ms;
    double guideRate = GuideRateNP[AXIS_DE].getValue() * LFAST_CONSTANTS::SiderealRate_degpersec;
    double guideManeuverTotalDistanceDeg = guideRate * guideDuration_ms / 1000.0;
    double guideStepSizeDeg = guideManeuverTotalDistanceDeg / numGuideSteps;
    m_EqSkyGuideDelta.rightascension = guideStepSizeDeg;
    guideManeuverActive = true;
    m_WETimer.start(abs(guideDuration_ms));

    return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double LFAST_Mount::GetSlewRate()
{
    // LOG_DEBUG("LFAST_Mount::GetSlewRate");
    ISwitch *Switch = IUFindOnSwitch(&SlewRateSP);
    return *(static_cast<double *>(Switch->aux));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ReadScopeStatus()
{
    LOG_DEBUG("ReadScopeStatus");

    // Update the state switch
    // int stateIndex = IUFindOnSwitchIndex(&TrackStateSP);
    // TrackStateSP[stateIndex].s = ISS_OFF;
    // TrackStateSP[TrackState].s = ISS_ON;
    // IDSetSwitch(&TrackStateSP, nullptr);

    // Calculate new RA DEC
    if (updatePointingCoordinates())
    {
        AzAltCoordsNP.apply();
    }
    switch (TrackState)
    {
    case SCOPE_SLEWING:
        if (homingRoutineActive)
        {
            if (!altHomingComplete)
            {
                altHomingComplete = AltitudeAxis->isHomingComplete();
                if (altHomingComplete)
                    LOG_INFO("Altitude Homing Complete.");
            }
            if (!azHomingComplete)
            {
                azHomingComplete = AzimuthAxis->isHomingComplete();
                if (azHomingComplete)
                    LOG_INFO("Azimuth Homing Complete.");
            }

            if (altHomingComplete && azHomingComplete)
            {
                HomeSP.setState(IPS_OK);
                HomeSP.apply();
                AzAltCoordsNP.setState(IPS_OK);
                AzAltCoordsNP.apply();
                AltitudeAxis->resetHomingRoutine();
                AzimuthAxis->resetHomingRoutine();

                homingRoutineActive = false;

                TrackState = SCOPE_IDLE;
                LOG_INFO("Homing Routine Complete.");
            }
        }
        else
        {
            if(!manualMotionActive)
            {
                if (AltitudeAxis->isSlewComplete() && AzimuthAxis->isSlewComplete())
                {
                    LOG_INFO("Slew Maneuver Complete.");
                    if (ISS_ON == IUFindSwitch(&CoordSP, "TRACK")->s)
                    {
                        TrackState = SCOPE_TRACKING;
                    }
                    else
                    {
                        TrackState = SCOPE_IDLE;
                    }
                }
            }
        }
        break;
    case SCOPE_PARKING:
    {
        bool azParked = AzimuthAxis->isSlewComplete();
        bool altParked = AltitudeAxis->isSlewComplete();
        if (azParked && altParked)
            TrackState = SCOPE_PARKED;
    }
    break;
    default:
        break;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::TimerHit()
{
    static struct timeval prevTime
    {
        0, 0
    }; // previous system time
    struct timeval currTime
    {
        0, 0
    }; // new system time

    gettimeofday(&currTime, nullptr);

    if (prevTime.tv_sec == 0 && prevTime.tv_usec == 0)
        prevTime = currTime;

    double dt; // Elapsed time in seconds since last tick
    dt = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_usec - prevTime.tv_usec) * 1e-6;
    prevTime = currTime;

    // This calls ReadScopeStatus()
    INDI::Telescope::TimerHit();

    if (TelemetryDownsampleNP[0].value >= TraceThisTickCount++)
    {
        TraceThisTick = true;
        TraceThisTickCount = 0;
    }

    // if(TraceThisTick)
    //     LOGF_TM("TimerHit: [dt: %.6f]", dt);

    std::string stateStr = {0};
    INDI::IHorizontalCoordinates altAzTgtPosn{0, 0};
    INDI::IHorizontalCoordinates altAzTgtRate{0, 0};

    switch (TrackState)
    {
    case SCOPE_IDLE:
        if (PrevTrackState != SCOPE_IDLE)
        {
            LOG_INFO("TimerHit: SCOPE_IDLE");
            PrevTrackState = SCOPE_IDLE;
            try
            {
                AltitudeAxis->slowStop();
                AzimuthAxis->slowStop();
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("TimerHit Error (SCOPE_IDLE):  %s", e.what());
            }
        }
        else
        {
            try
            {
                AltitudeAxis->checkDriveStatus();
                AzimuthAxis->checkDriveStatus();
            }
            catch (const std::exception &e)
            {
                LOGF_WARN("TimerHit Warning (SCOPE_IDLE):  %s", e.what());
            }
        }
        break;
    case SCOPE_SLEWING:
        if (PrevTrackState != SCOPE_SLEWING)
        {
            if (manualMotionActive)
                LOG_INFO("TimerHit: SCOPE_SLEWING (Manual)");
            else
                LOG_INFO("TimerHit: SCOPE_SLEWING (Goto)");
            PrevTrackState = SCOPE_SLEWING;
        }
        if (homingRoutineActive)
        {
            m_SkyGuideOffset = {0, 0};
            AltitudeAxis->serviceHomingRoutine();
            AzimuthAxis->serviceHomingRoutine();
        }
        else
        {
            if (manualMotionActive)
            {
                // LOG_INFO("Manual slewing active");
                try
                {
                    AltitudeAxis->updateControlLoops(dt, MANUAL_SLEW);
                    AzimuthAxis->updateControlLoops(dt, MANUAL_SLEW);
                }
                catch (const std::exception &e)
                {
                    LOGF_ERROR("TimerHit Error (MANUAL_SLEW):  %s", e.what());
                    TrackState = SCOPE_IDLE;
                }
            }
            else
            {
                m_SkyGuideOffset = {0, 0};
                altAzTgtPosn = getTrackingTargetAltAzPosition();
                AltitudeAxis->updateTrackCommands(altAzTgtPosn.altitude);
                AzimuthAxis->updateTrackCommands(altAzTgtPosn.azimuth);
                try
                {
                    AltitudeAxis->updateControlLoops(dt, SLEWING_TO_POSN);
                    AzimuthAxis->updateControlLoops(dt, SLEWING_TO_POSN);
                }
                catch (const std::exception &e)
                {
                    LOGF_ERROR("TimerHit Error (SCOPE_SLEWING):  %s", e.what());
                    TrackState = SCOPE_IDLE;
                }
            }
        }
        break;
    case SCOPE_TRACKING:
        if (PrevTrackState != SCOPE_TRACKING)
        {
            LOG_INFO("TimerHit: SCOPE_TRACKING");
            PrevTrackState = SCOPE_TRACKING;
        }
        altAzTgtPosn = getTrackingTargetAltAzPosition();
        altAzTgtRate = getSiderealTargetAltAzRates();
        // AltitudeAxis->updateTrackCommands(altAzTgtPosn.altitude);
        // AzimuthAxis->updateTrackCommands(altAzTgtPosn.azimuth);
        AltitudeAxis->updateTrackCommands(altAzTgtPosn.altitude, altAzTgtRate.altitude);
        AzimuthAxis->updateTrackCommands(altAzTgtPosn.azimuth, altAzTgtRate.azimuth);
        try
        {
            AltitudeAxis->updateControlLoops(dt, TRACKING_COMMAND);
            AzimuthAxis->updateControlLoops(dt, TRACKING_COMMAND);
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("TimerHit Error (SCOPE_TRACKING):  %s", e.what());
            TrackState = SCOPE_IDLE;
        }
        break;
    case SCOPE_PARKING:
    {
        if (PrevTrackState != SCOPE_PARKING)
        {
            LOG_INFO("TimerHit: SCOPE_PARKING");
            PrevTrackState = SCOPE_PARKING;
        }

        try
        {
            AzimuthAxis->updateControlLoops(dt, SLEWING_TO_POSN);
            AltitudeAxis->updateControlLoops(dt, SLEWING_TO_POSN);
        }
        catch (const std::exception &e)
        {
            LOGF_ERROR("TimerHit Error (SCOPE_PARKING:  %s", e.what());
            TrackState = SCOPE_IDLE;
        }
        break;
    }
    case SCOPE_PARKED:
        if (PrevTrackState == SCOPE_PARKING)
        {
            try
            {
                AzimuthAxis->slowStop();
                AltitudeAxis->slowStop();
                TrackState = SCOPE_PARKED;
                LOG_INFO("Scope Parked.");
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("TimerHit Error (SCOPE_PARKED):  %s", e.what());
                TrackState = SCOPE_IDLE;
            }
            SetParked(true);
            PrevTrackState = SCOPE_PARKED;
        }
        break;
    }

    if (TrackState == SCOPE_SLEWING || TrackState == SCOPE_TRACKING)
    {
        if (TraceThisTick)
            tmLogMountStates();
    }
    TraceThisTick = false;
}

void LFAST_Mount::hexDump(char *buf, const char *data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::tmLogMountStates()
{
    double azPosnFb, altPosnFb, azRateFb, altRateFb, azPosnCmd, altPosnCmd;
    try
    {
        altPosnCmd = AltitudeAxis->getPositionCommand();
        azPosnCmd = AzimuthAxis->getPositionCommand();

        altPosnFb = AltitudeAxis->getPositionState();
        azPosnFb = AzimuthAxis->getPositionState();

        altRateFb = AltitudeAxis->getVelocityState();
        azRateFb = AzimuthAxis->getVelocityState();
    }
    catch (const std::exception &e)
    {
        LOGF_ERROR("tmLogMountStates Error: %s", e.what());
    }

    // double altPosnErr = altPosnCmd - altPosnFb;
    // double azPosnErr = azPosnCmd - azPosnFb;

    LOGF_TM("POSN CMD: [ALT: %6.4f], [AZ: %6.4f]", altPosnCmd, azPosnCmd);
    LOGF_TM("POSN FB: [ALT: %6.4f], [AZ: %6.4f]", altPosnFb, azPosnFb);
    // LOGF_TM("POSN ERR: [ALT: %6.4f], [AZ: %6.4f]", altPosnErr, azPosnErr);
    LOGF_TM("RATE CMD: [ALT: %6.4f], [AZ: %6.4f]", AltitudeAxis->getVelocityCommand(), AzimuthAxis->getVelocityCommand());
    LOGF_TM("RATE FB: [ALT: %6.4f], [AZ: %6.4f]", altRateFb, azRateFb);
}