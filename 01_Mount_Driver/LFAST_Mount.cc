/*!
 * \file lfast_mount_driver.cpp
 *
 * \author Roger James
 * \author Jasem Mutlaq
 * \author Gerry Rozema
 * \author Jean-Luc Geehalel
 * \date 13th November 2013
 *
 * Updated on 2020-12-01 by Jasem Mutlaq
 * Updated on 2021-11-20 by Jasem Mutlaq:
 *  + Fixed tracking.
 *  + Added iterative GOTO.
 *  + Simplified driver and logging.
 *
 * This file contains an implementation in C++ of the Skywatcher API.
 * It is based on work from four sources.
 * A C++ implementation of the API by Roger James.
 * The indi_eqmod driver by Jean-Luc Geehalel.
 * The synscanmount driver by Gerry Rozema.
 * The C# implementation published by Skywatcher/Synta
 */

#include "LFAST_Mount.h"

#include "indicom.h"
#include "connectionplugins/connectiontcp.h"
#include "alignment/DriverCommon.h"
#include "connectionplugins/connectionserial.h"

#include <chrono>
#include <thread>
#include <exception>

#include <sys/stat.h>

using namespace INDI::AlignmentSubsystem;

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

const char azLabel[] = "Az. Axis";
const char altLabel[] = "Alt. Axis";

// We declare an auto pointer to LFAST_Mount.
static std::unique_ptr<LFAST_Mount> LFAST_MountPtr(new LFAST_Mount());

/* Preset Slew Speeds */
#define SLEWMODES 9
static double SlewSpeeds[SLEWMODES] = {1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0};

const double constexpr default_park_posn_az = 00.0;
const double constexpr default_park_posn_alt = 0.0;

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
LFAST_Mount::LFAST_Mount()
{
    // Set up the logging pointer in SkyWatcherAPI
    // pChildTelescope  = this;
    SetTelescopeCapability(SCOPE_CAPABILITIES,
                           SLEWMODES);

    setVersion(0, 4);
    setTelescopeConnection(CONNECTION_TCP);
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
    AltitudeAxis = new SlewDrive(altLabel);
    AzimuthAxis = new SlewDrive(azLabel);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Handshake()
{
    DEBUG(DBG_SCOPE, "LFAST_Mount::Handshake");
    if (!getActiveConnection()->name().compare("CONNECTION_TCP"))
    {
        tty_set_generic_udp_format(1);
    }

    // bool Result = InitMount();
    // DEBUGF(DBG_SCOPE, "LFAST_Mount::Handshake - Result: %d", Result);
    return true;
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
    // Allow the base class to initialise its visible before connection properties
    INDI::Telescope::initProperties();

    for (int i = 0; i < SlewRateSP.nsp; ++i)
    {
        sprintf(SlewRateSP.sp[i].label, "%.fx", SlewSpeeds[i]);
        SlewRateSP.sp[i].aux = &SlewSpeeds[i];
    }
    strncpy(SlewRateSP.sp[SlewRateSP.nsp - 1].name, "SLEW_MAX", MAXINDINAME);

    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    // AddTrackMode("TRACK_SOLAR", "Solar");
    // AddTrackMode("TRACK_LUNAR", "Lunar");

    // Add default properties
    addDebugControl();
    addConfigurationControl();

    // Add alignment properties
    InitAlignmentProperties(this);

    // Force the alignment system to always be on
    getSwitch("ALIGNMENT_SUBSYSTEM_ACTIVE")->sp[0].s = ISS_ON;

    // Set up property variables
    // IUFillText(&BasicMountInfoT[MOTOR_CONTROL_FIRMWARE_VERSION], "MOTOR_CONTROL_FIRMWARE_VERSION",
    //            "Motor control firmware version", "-");
    // IUFillText(&BasicMountInfoT[MOUNT_CODE], "MOUNT_CODE", "Mount code", "-");
    // IUFillText(&BasicMountInfoT[MOUNT_NAME], "MOUNT_NAME", "Mount name", "-");
    // IUFillText(&BasicMountInfoT[IS_DC_MOTOR], "IS_DC_MOTOR", "Is DC motor", "-");
    // IUFillTextVector(&BasicMountInfoTP, BasicMountInfoT, 4, getDeviceName(), "BASIC_MOUNT_INFO",
    //                  "Basic mount information", MountInfoTab, IP_RO, 60, IPS_IDLE);

    // IUFillNumber(&AxisOneInfoN[MICROSTEPS_PER_REVOLUTION], "MICROSTEPS_PER_REVOLUTION", "Microsteps per revolution",
    //              "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneInfoN[STEPPER_CLOCK_FREQUENCY], "STEPPER_CLOCK_FREQUENCY", "Stepper clock frequency", "%.0f", 0,
    //              0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneInfoN[HIGH_SPEED_RATIO], "HIGH_SPEED_RATIO", "High speed ratio", "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneInfoN[MICROSTEPS_PER_WORM_REVOLUTION], "MICROSTEPS_PER_WORM_REVOLUTION",
    //              "Microsteps per worm revolution", "%.0f", 0, 0xFFFFFF, 1, 0);

    // MountInfoTab
    AzPosnNP[COMMAND].fill("AZ_POSITION_COMMAND", "Az Posn Cmd [deg]", "%6.4f", 0, 360, 0.001, 0);
    AzPosnNP[FEEDBACK].fill("AZ_POSITION_FEEDBACK", "Az Posn Fb [deg]", "%6.4f", 0, 360, 0.001, 0);
    AzPosnNP.fill(getDeviceName(), "AZ_POSITION", "Az Posn", MountInfoTab, IP_RO, 60, IPS_IDLE);

    AltPosnNP[COMMAND].fill("ALT_POSITION_COMMAND", "Alt Posn Cmd [deg]", "%6.4f", -90, 90, 0.001, 0);
    AltPosnNP[FEEDBACK].fill("ALT_POSITION_FEEDBACK", "Alt Posn Fb [deg]", "%6.4f", -90, 90, 0.001, 0);
    AltPosnNP.fill(getDeviceName(), "ALT_POSITION", "Alt Posn", MountInfoTab, IP_RO, 60, IPS_IDLE);

    AzRateNP[COMMAND].fill("AZ_RATE_COMMAND", "Az Rate Cmd [deg/s]", "%6.4f", -100000, 100000, 0.001, 0);
    AzRateNP[FEEDBACK].fill("AZ_RATE_FEEDBACK", "Az Rate Fb [deg/s]", "%6.4f", -100000, 100000, 0.001, 0);
    AzRateNP.fill(getDeviceName(), "AZ_RATE", "Az Rate", MountInfoTab, IP_RO, 60, IPS_IDLE);

    AltRateNP[COMMAND].fill("ALT_RATE_COMMAND", "Alt Rate Cmd [deg/s]", "%6.4f", -100000, 100000, 0.001, 0);
    AltRateNP[FEEDBACK].fill("ALT_RATE_FEEDBACK", "Alt Rate Fb [deg]/s", "%6.4f", -100000, 100000, 0.001, 0);
    AltRateNP.fill(getDeviceName(), "ALT_RATE", "Alt Rate", MountInfoTab, IP_RO, 60, IPS_IDLE);

    // IUFillNumberVector(&AxisOneInfoNP, AxisOneInfoN, 4, getDeviceName(), "AXIS_ONE_INFO", "Axis one information",
    //                    MountInfoTab, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&AxisOneStateS[FULL_STOP], "FULL_STOP", "FULL_STOP", ISS_OFF);
    IUFillSwitch(&AxisOneStateS[SLEWING], "SLEWING", "SLEWING", ISS_OFF);
    IUFillSwitch(&AxisOneStateS[SLEWING_TO], "SLEWING_TO", "SLEWING_TO", ISS_OFF);
    IUFillSwitch(&AxisOneStateS[SLEWING_FORWARD], "SLEWING_FORWARD", "SLEWING_FORWARD", ISS_OFF);
    IUFillSwitch(&AxisOneStateS[HIGH_SPEED], "HIGH_SPEED", "HIGH_SPEED", ISS_OFF);
    IUFillSwitch(&AxisOneStateS[NOT_INITIALISED], "NOT_INITIALISED", "NOT_INITIALISED", ISS_ON);
    IUFillSwitchVector(&AxisOneStateSP, AxisOneStateS, 6, getDeviceName(), "AXIS_ONE_STATE", "Axis one state",
                       MountInfoTab, IP_RO, ISR_NOFMANY, 60, IPS_IDLE);

    // IUFillNumber(&AxisTwoInfoN[MICROSTEPS_PER_REVOLUTION], "MICROSTEPS_PER_REVOLUTION", "Microsteps per revolution",
    //              "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoInfoN[STEPPER_CLOCK_FREQUENCY], "STEPPER_CLOCK_FREQUENCY", "Step timer frequency", "%.0f", 0,
    //              0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoInfoN[HIGH_SPEED_RATIO], "HIGH_SPEED_RATIO", "High speed ratio", "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoInfoN[MICROSTEPS_PER_WORM_REVOLUTION], "MICROSTEPS_PER_WORM_REVOLUTION",
    //              "Microsteps per worm revolution", "%.0f", 0, 0xFFFFFF, 1, 0);

    // IUFillNumberVector(&AxisTwoInfoNP, AxisTwoInfoN, 4, getDeviceName(), "AXIS_TWO_INFO", "Axis two information",
    //                    MountInfoTab, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&AxisTwoStateS[FULL_STOP], "FULL_STOP", "FULL_STOP", ISS_OFF);
    IUFillSwitch(&AxisTwoStateS[SLEWING], "SLEWING", "SLEWING", ISS_OFF);
    IUFillSwitch(&AxisTwoStateS[SLEWING_TO], "SLEWING_TO", "SLEWING_TO", ISS_OFF);
    IUFillSwitch(&AxisTwoStateS[SLEWING_FORWARD], "SLEWING_FORWARD", "SLEWING_FORWARD", ISS_OFF);
    IUFillSwitch(&AxisTwoStateS[HIGH_SPEED], "HIGH_SPEED", "HIGH_SPEED", ISS_OFF);
    IUFillSwitch(&AxisTwoStateS[NOT_INITIALISED], "NOT_INITIALISED", "NOT_INITIALISED", ISS_ON);
    IUFillSwitchVector(&AxisTwoStateSP, AxisTwoStateS, 6, getDeviceName(), "AXIS_TWO_STATE", "Axis two state",
                       MountInfoTab, IP_RO, ISR_NOFMANY, 60, IPS_IDLE);

    // IUFillNumber(&AxisOneEncoderValuesN[RAW_MICROSTEPS], "RAW_MICROSTEPS", "Raw Microsteps", "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneEncoderValuesN[MICROSTEPS_PER_ARCSEC], "MICROSTEPS_PER_ARCSEC", "Microsteps/arcsecond",
    //              "%.4f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneEncoderValuesN[OFFSET_FROM_INITIAL], "OFFSET_FROM_INITIAL", "Offset from initial", "%.0f", 0,
    //              0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisOneEncoderValuesN[DEGREES_FROM_INITIAL], "DEGREES_FROM_INITIAL", "Degrees from initial", "%.2f",
    //              -1000.0, 1000.0, 1, 0);

    // IUFillNumberVector(&AxisOneEncoderValuesNP, AxisOneEncoderValuesN, 4, getDeviceName(), "AXIS1_ENCODER_VALUES",
    //                    "Axis 1 Encoder values", MountInfoTab, IP_RO, 60, IPS_IDLE);

    // IUFillNumber(&AxisTwoEncoderValuesN[RAW_MICROSTEPS], "RAW_MICROSTEPS", "Raw Microsteps", "%.0f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoEncoderValuesN[MICROSTEPS_PER_ARCSEC], "MICROSTEPS_PER_ARCSEC", "Microsteps/arcsecond",
    //              "%.4f", 0, 0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoEncoderValuesN[OFFSET_FROM_INITIAL], "OFFSET_FROM_INITIAL", "Offset from initial", "%.0f", 0,
    //              0xFFFFFF, 1, 0);
    // IUFillNumber(&AxisTwoEncoderValuesN[DEGREES_FROM_INITIAL], "DEGREES_FROM_INITIAL", "Degrees from initial", "%.2f",
    //              -1000.0, 1000.0, 1, 0);

    // IUFillNumberVector(&AxisTwoEncoderValuesNP, AxisTwoEncoderValuesN, 4, getDeviceName(), "AXIS2_ENCODER_VALUES",
    //                    "Axis 2 Encoder values", MountInfoTab, IP_RO, 60, IPS_IDLE);
    // Register any visible before connection properties

    // Slew modes
    IUFillSwitch(&SlewModesS[SLEW_SILENT], "SLEW_SILENT", "Silent", ISS_OFF);
    IUFillSwitch(&SlewModesS[SLEW_NORMAL], "SLEW_NORMAL", "Normal", ISS_ON);
    IUFillSwitchVector(&SlewModesSP, SlewModesS, 2, getDeviceName(), "TELESCOPE_MOTION_SLEWMODE", "Slew Mode",
                       MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // SoftPEC modes
    IUFillSwitch(&SoftPECModesS[SOFTPEC_ENABLED], "SOFTPEC_ENABLED", "Enable for tracking", ISS_OFF);
    IUFillSwitch(&SoftPECModesS[SOFTPEC_DISABLED], "SOFTPEC_DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&SoftPECModesSP, SoftPECModesS, 2, getDeviceName(), "TELESCOPE_MOTION_SOFTPECMODE",
                       "SoftPEC Mode", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // SoftPEC value for tracking mode
    IUFillNumber(&SoftPecN, "SOFTPEC_VALUE", "degree/minute (Alt)", "%1.3f", 0.001, 1.0, 0.001, 0.009);
    IUFillNumberVector(&SoftPecNP, &SoftPecN, 1, getDeviceName(), "SOFTPEC", "SoftPEC Value", MOTION_TAB, IP_RW, 60,
                       IPS_IDLE);

    // Guiding rates for RA/DEC axes
    IUFillNumber(&GuidingRatesN[0], "GUIDERA_RATE", "arcsec/seconds (RA)", "%1.3f", 1.0, 6000.0, 1.0, 120.0);
    IUFillNumber(&GuidingRatesN[1], "GUIDEDEC_RATE", "arcsec/seconds (Dec)", "%1.3f", 1.0, 6000.0, 1.0, 120.0);
    IUFillNumberVector(&GuidingRatesNP, GuidingRatesN, 2, getDeviceName(), "GUIDE_RATES", "Guide Rates", MOTION_TAB,
                       IP_RW, 60, IPS_IDLE);

    // // AUX Encoders
    // AUXEncoderSP[INDI_ENABLED].fill("INDI_ENABLED", "Enabled", ISS_OFF);
    // AUXEncoderSP[INDI_DISABLED].fill("INDI_DISABLED", "Disabled", ISS_ON);
    // AUXEncoderSP.fill(getDeviceName(), "AUX_ENCODERS", "AUX Encoders", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // // Snap port
    // SnapPortSP[INDI_ENABLED].fill("INDI_ENABLED", "On", ISS_OFF);
    // SnapPortSP[INDI_DISABLED].fill("INDI_DISABLED", "Off", ISS_ON);
    // SnapPortSP.fill(getDeviceName(), "SNAP_PORT", "Snap Port", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Tracking Factor
    TrackFactorNP[AXIS_AZ].fill("AXIS_AZ", "Azimuth", "%.2f", 0.1, 5, 0.1, 1);
    TrackFactorNP[AXIS_ALT].fill("AXIS_ALT", "Altitude", "%.2f", 0.1, 5, 0.1, 1);
    TrackFactorNP.fill(getDeviceName(), "TRACK_FACTOR", "Track Factor", MOTION_TAB, IP_RW, 60, IPS_IDLE);

    // tcpConnection->setDefaultHost("192.168.4.1");
    // tcpConnection->setDefaultPort(11880);
    tcpConnection->setConnectionType(Connection::TCP::TYPE_UDP);

    if (strstr(getDeviceName(), "GTi"))
        // {
        //     setActiveConnection(tcpConnection);
        //     tcpConnection->setLANSearchEnabled(true);
        // }

        SetParkDataType(PARK_AZ_ALT_ENCODER);

    // Guiding support
    initGuiderProperties(getDeviceName(), GUIDE_TAB);
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    // Set default values in parent class
    IUFindNumber(&ScopeParametersNP, "TELESCOPE_APERTURE")->value = 200;
    IUFindNumber(&ScopeParametersNP, "TELESCOPE_FOCAL_LENGTH")->value = 2000;
    IUFindNumber(&ScopeParametersNP, "GUIDER_APERTURE")->value = 30;
    IUFindNumber(&ScopeParametersNP, "GUIDER_FOCAL_LENGTH")->value = 120;

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
        // It is for us
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
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        ProcessAlignmentNumberProperties(this, name, values, names, n);

        if (strcmp(name, "SOFTPEC") == 0)
        {
            SoftPecNP.s = IPS_OK;
            IUUpdateNumber(&SoftPecNP, values, names, n);
            IDSetNumber(&SoftPecNP, nullptr);
            return true;
        }

        if (strcmp(name, "GUIDE_RATES") == 0)
        {
            ResetGuidePulses();
            GuidingRatesNP.s = IPS_OK;
            IUUpdateNumber(&GuidingRatesNP, values, names, n);
            IDSetNumber(&GuidingRatesNP, nullptr);
            return true;
        }

        if (TrackFactorNP.isNameMatch(name))
        {
            TrackFactorNP.update(values, names, n);
            TrackFactorNP.setState(IPS_OK);
            TrackFactorNP.apply();
            saveConfig(true, TrackFactorNP.getName());
            return true;
        }

        // Let our driver do sync operation in park position
        if (strcmp(name, "EQUATORIAL_EOD_COORD") == 0)
        {
            double ra = -1;
            double dec = -100;

            for (int x = 0; x < n; x++)
            {
                INumber *eqp = IUFindNumber(&EqNP, names[x]);
                if (eqp == &EqN[AXIS_RA])
                {
                    ra = values[x];
                }
                else if (eqp == &EqN[AXIS_DE])
                {
                    dec = values[x];
                }
            }
            if ((ra >= 0) && (ra <= 24) && (dec >= -90) && (dec <= 90))
            {
                ISwitch *sw = IUFindSwitch(&CoordSP, "SYNC");

                if (sw != nullptr && sw->s == ISS_ON && isParked())
                {
                    return Sync(ra, dec);
                }
            }
        }

        processGuiderProperties(name, values, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // // Auxiliary Encoders
        // if (AUXEncoderSP.isNameMatch(name))
        // {
        //     AUXEncoderSP.update(states, names, n);
        //     AUXEncoderSP.setState(IPS_OK);
        //     AUXEncoderSP.apply();
        //     auto enabled = AUXEncoderSP.findOnSwitchIndex() == INDI_ENABLED;
        //     TurnRAEncoder(enabled);
        //     TurnDEEncoder(enabled);
        //     saveConfig(true, AUXEncoderSP.getName());
        //     return true;
        // }

        // Snap Port
        // if (SnapPortSP.isNameMatch(name))
        // {
        //     SnapPortSP.update(states, names, n);
        //     auto enabled = SnapPortSP.findOnSwitchIndex() == INDI_ENABLED;
        //     toggleSnapPort(enabled);
        //     if (enabled)
        //         LOG_INFO("Toggling snap port on...");
        //     else
        //         LOG_INFO("Toggling snap port off...");
        //     SnapPortSP.setState(enabled ? IPS_OK : IPS_IDLE);
        //     SnapPortSP.apply();
        //     return true;
        // }
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        ProcessAlignmentTextProperties(this, name, texts, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Goto(double ra, double dec)
{
    LOG_INFO("LFAST_Mount::Goto");
    if (m_IterativeGOTOPending)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, m_SkyCurrentRADE.rightascension, 2, 3600);
        fs_sexa(DecStr, m_SkyCurrentRADE.declination, 2, 3600);
        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Iterative GOTO RA %lf DEC %lf (Current Sky RA %s DE %s)", ra, dec, RAStr,
               DecStr);
    }
    else
    {
        if (TrackState != SCOPE_IDLE)
            Abort();

        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "GOTO RA %lf DEC %lf", ra, dec);

        if (IUFindSwitch(&CoordSP, "TRACK")->s == ISS_ON || IUFindSwitch(&CoordSP, "SLEW")->s == ISS_ON)
        {
            char RAStr[32], DecStr[32];
            fs_sexa(RAStr, ra, 2, 3600);
            fs_sexa(DecStr, dec, 2, 3600);
            m_SkyTrackingTarget.rightascension = ra;
            m_SkyTrackingTarget.declination = dec;
            LOGF_INFO("Goto target RA %s DEC %s", RAStr, DecStr);
        }
    }

    INDI::IHorizontalCoordinates AltAz{0, 0};
    TelescopeDirectionVector TDV;

    // Transform Celestial to Telescope coordinates.
    // We have no good way to estimate how long will the mount takes to reach target (with deceleration,
    // and not just speed). So we will use iterative GOTO once the first GOTO is complete.
    if (TransformCelestialToTelescope(ra, dec, 0.0, TDV))
    {
        INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
        AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
        INDI::HorizontalToEquatorial(&AltAz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);

        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, EquatorialCoordinates.rightascension, 2, 3600);
        fs_sexa(DecStr, EquatorialCoordinates.declination, 2, 3600);

        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Sky -> Mount RA %s DE %s (TDV x %lf y %lf z %lf)", RAStr, DecStr, TDV.x,
               TDV.y, TDV.z);
    }
    else
    {
        // Try a conversion with the stored observatory position if any
        INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
        EquatorialCoordinates.rightascension = ra;
        EquatorialCoordinates.declination = dec;
        INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &AltAz);
        TDV = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
        switch (GetApproximateMountAlignment())
        {
        case ZENITH:
            break;

        case NORTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 minus
            // the (positive)observatory latitude. The vector itself is rotated anticlockwise
            TDV.RotateAroundY(m_Location.latitude - 90.0);
            break;

        case SOUTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 plus
            // the (negative)observatory latitude. The vector itself is rotated clockwise
            TDV.RotateAroundY(m_Location.latitude + 90.0);
            break;
        }
        AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
    }

    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT,
    //        "Sky -> Mount AZ %lf° (%ld) AL %lf° (%ld)",
    //        AltAz.azimuth,
    //        DegreesToMicrosteps(AXIS1, AltAz.azimuth),
    //        AltAz.altitude,
    //        DegreesToMicrosteps(AXIS2, AltAz.altitude));

    // Update the current encoder positions
    // GetEncoder(AXIS1);
    // GetEncoder(AXIS2);

    // long AzimuthOffsetMicrosteps = DegreesToMicrosteps(AXIS1,
    //                                                    AltAz.azimuth) +
    //                                ZeroPositionEncoders[AXIS1] - CurrentEncoders[AXIS1];
    // long AltitudeOffsetMicrosteps = DegreesToMicrosteps(AXIS2,
    //                                                     AltAz.altitude) +
    //                                 ZeroPositionEncoders[AXIS2] - CurrentEncoders[AXIS2];

    // if (AzimuthOffsetMicrosteps > MicrostepsPerRevolution[AXIS1] / 2)
    // {
    //     // Going the long way round - send it the other way
    //     AzimuthOffsetMicrosteps -= MicrostepsPerRevolution[AXIS1];
    // }

    // // Do I need to take out any complete revolutions before I do this test?
    // if (AltitudeOffsetMicrosteps > MicrostepsPerRevolution[AXIS2] / 2)
    // {
    //     // Going the long way round - send it the other way
    //     AltitudeOffsetMicrosteps -= MicrostepsPerRevolution[AXIS2];
    // }

    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Current Axis1 %ld microsteps (Zero %ld) Axis2 %ld microsteps (Zero %ld)",
    //        CurrentEncoders[AXIS1], ZeroPositionEncoders[AXIS1], CurrentEncoders[AXIS2], ZeroPositionEncoders[AXIS2]);
    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Azimuth offset %ld microsteps | Altitude offset %ld microsteps",
    //        AzimuthOffsetMicrosteps, AltitudeOffsetMicrosteps);

    // SilentSlewMode = (IUFindSwitch(&SlewModesSP, "SLEW_SILENT") != nullptr && IUFindSwitch(&SlewModesSP, "SLEW_SILENT")->s == ISS_ON);

    // SlewTo(AXIS1, AzimuthOffsetMicrosteps);
    // SlewTo(AXIS2, AltitudeOffsetMicrosteps);

    DEBUGF(DBG_SCOPE, "Goto - Scope reference frame target altitude %lf azimuth %lf", AltAz.altitude,
           AltAz.azimuth);

    // TODO: Try/Catch
    AltitudeAxis->updateTrackCommands(AltAz.altitude);
    AzimuthAxis->updateTrackCommands(AltAz.azimuth);
    TrackState = SCOPE_SLEWING;

    return true;
}
bool LFAST_Mount::SetSlewRate(int index)
{
    double mult = SlewSpeeds[index];
    double rate = mult * SIDEREALRATE_DPS;
    AzimuthAxis->setSlewRate(mult);
    AltitudeAxis->setSlewRate(mult);
    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::ISGetProperties(const char *dev)
{
    INDI::Telescope::ISGetProperties(dev);

    if (isConnected())
    {
        // Fill in any real values now available MCInit should have been called already
        UpdateDetailedMountInformation(false);

        // Define our connected only properties to the base driver
        // e.g. defineProperty(MyNumberVectorPointer);
        // This will register our properties and send a IDDefXXXX mewssage to any connected clients
        // defineProperty(&BasicMountInfoTP);
        // defineProperty(&AxisOneInfoNP);
        defineProperty(&AxisOneStateSP);
        // defineProperty(&AxisTwoInfoNP);
        defineProperty(&AxisTwoStateSP);
        // defineProperty(&AxisOneEncoderValuesNP);
        // defineProperty(&AxisTwoEncoderValuesNP);
        defineProperty(&SlewModesSP);
        defineProperty(&SoftPECModesSP);
        defineProperty(&SoftPecNP);
        defineProperty(&GuidingRatesNP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double LFAST_Mount::GetSlewRate()
{
    ISwitch *Switch = IUFindOnSwitch(&SlewRateSP);
    return *(static_cast<double *>(Switch->aux));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    double speed =
        (dir == DIRECTION_NORTH) ? GetSlewRate() * LOW_SPEED_MARGIN / 2 : -GetSlewRate() * LOW_SPEED_MARGIN / 2;
    const char *dirStr = (dir == DIRECTION_NORTH) ? "North" : "South";

    switch (command)
    {
    case MOTION_START:
        DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
        // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
        AltitudeAxis->updateRateOffset(speed);
        AzimuthAxis->updateRateOffset(speed);
        m_ManualMotionActive = true;
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
    double speed =
        (dir == DIRECTION_WEST) ? -GetSlewRate() * LOW_SPEED_MARGIN / 2 : GetSlewRate() * LOW_SPEED_MARGIN / 2;
    const char *dirStr = (dir == DIRECTION_WEST) ? "West" : "East";

    switch (command)
    {
    case MOTION_START:
        DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
        // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
        AzimuthAxis->updateRateOffset(speed);
        m_ManualMotionActive = true;
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
bool LFAST_Mount::Park()
{
    LOG_INFO("LFAST_Mount::Park");
    // Move the telescope to the desired position
    // if (IUFindSwitch(&SlewModesSP, "SLEW_SILENT") != nullptr && IUFindSwitch(&SlewModesSP, "SLEW_SILENT")->s == ISS_ON)
    // {
    //     SilentSlewMode = true;
    // }
    // else
    // {
    //     SilentSlewMode = false;
    // }

    AltitudeAxis->updateTrackCommands(default_park_posn_alt);
    AzimuthAxis->updateTrackCommands(default_park_posn_az);

    TrackState = SCOPE_PARKING;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::UnPark()
{
    SetParked(false);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::SetTrackEnabled(bool enabled)
{
    if (enabled)
    {
        TrackState = SCOPE_TRACKING;
        resetTracking();
        m_SkyTrackingTarget.rightascension = EqN[AXIS_RA].value;
        m_SkyTrackingTarget.declination = EqN[AXIS_DE].value;
    }
    else
        TrackState = SCOPE_IDLE;

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::ReadScopeStatus()
{
    double azPosnFb, altPosnFb;
    // double azPosnFb, altPosnFb, azRateFb, altRateFb;
    try
    {
        azPosnFb = AzimuthAxis->getPositionFeedback();
        altPosnFb = AltitudeAxis->getPositionFeedback();
        // azRateFb = AzimuthAxis->getVelocityFeedback();
        // altRateFb = AltitudeAxis->getVelocityFeedback();
    }
    catch (const std::exception &e)
    {
        LOG_ERROR(e.what());
    }

    bool resetTrackingTimers = false;

    // Calculate new RA DEC
    INDI::IHorizontalCoordinates AltAzFB{0, 0};
    AltAzFB.altitude = altPosnFb;
    AltAzFB.azimuth = azPosnFb;

    AltAzFB.azimuth = azPosnFb;
    AltAzFB.altitude = altPosnFb;

    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Axis1 encoder %ld (Zero %ld) -> AZ %lf°",
    //        CurrentEncoders[AXIS1], ZeroPositionEncoders[AXIS1], AltAz.azimuth);
    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Axis2 encoder %ld (Zero %ld) -> AL %lf°",
    //        CurrentEncoders[AXIS2], ZeroPositionEncoders[AXIS2], AltAz.altitude);

    // Update current horizontal coords.
    m_MountAltAz = AltAzFB;

    // Get equatorial coords.
    getCurrentRADE(AltAzFB, m_SkyCurrentRADE);
    char RAStr[32], DecStr[32];
    fs_sexa(RAStr, m_SkyCurrentRADE.rightascension, 2, 3600);
    fs_sexa(DecStr, m_SkyCurrentRADE.declination, 2, 3600);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Sky RA %s DE %s", RAStr, DecStr);

    if (TrackState == SCOPE_SLEWING)
    {
        if (AzimuthAxis->isStopped() && AltitudeAxis->isStopped())
        {
            // If iterative GOTO was already engaged, stop it.
            if (m_IterativeGOTOPending)
                m_IterativeGOTOPending = false;
            // If not, then perform the iterative GOTO once more.
            else
            {
                m_IterativeGOTOPending = true;
                return Goto(m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination);
            }

            if (ISS_ON == IUFindSwitch(&CoordSP, "TRACK")->s)
            {
                // Goto has finished start tracking
                TrackState = SCOPE_TRACKING;
                resetTrackingTimers = true;
                LOG_INFO("Tracking started.");
            }
            else
            {
                TrackState = SCOPE_IDLE;
            }
        }
    }
    else if (TrackState == SCOPE_PARKING)
    {
        if (AzimuthAxis->isStopped() && AltitudeAxis->isStopped())
        {
            LOG_INFO("PARK DONE.");
            AltitudeAxis->slowStop();
            AzimuthAxis->slowStop();
            SetParked(true);
        }
    }

    if (resetTrackingTimers)
        resetTracking();

    NewRaDec(m_SkyCurrentRADE.rightascension, m_SkyCurrentRADE.declination);
    UpdateDetailedMountInformation(true);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
// bool LFAST_Mount::getCurrentAltAz(INDI::IHorizontalCoordinates &altaz)
// {
//     // Update Axis Position
//     if (GetEncoder(AXIS1) && GetEncoder(AXIS2))
//     {
//         altaz.azimuth = range360(MicrostepsToDegrees(AXIS1, CurrentEncoders[AXIS1] - ZeroPositionEncoders[AXIS1]));
//         altaz.altitude = MicrostepsToDegrees(AXIS2, CurrentEncoders[AXIS2] - ZeroPositionEncoders[AXIS2]);
//         return true;
//     }

//     return false;
// }

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::getCurrentRADE(INDI::IHorizontalCoordinates altaz, INDI::IEquatorialCoordinates &rade)
{
    TelescopeDirectionVector TDV = TelescopeDirectionVectorFromAltitudeAzimuth(altaz);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);

    double RightAscension, Declination;
    if (!TransformTelescopeToCelestial(TDV, RightAscension, Declination))
    {
        TelescopeDirectionVector RotatedTDV(TDV);
        switch (GetApproximateMountAlignment())
        {
        case ZENITH:
            break;

        case NORTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 minus
            // the (positive)observatory latitude. The vector itself is rotated clockwise
            RotatedTDV.RotateAroundY(90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, altaz);
            break;

        case SOUTH_CELESTIAL_POLE:
            // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 plus
            // the (negative)observatory latitude. The vector itself is rotated anticlockwise
            RotatedTDV.RotateAroundY(-90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, altaz);
            break;
        }

        INDI::IEquatorialCoordinates EquatorialCoordinates;
        INDI::HorizontalToEquatorial(&altaz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
        RightAscension = EquatorialCoordinates.rightascension;
        Declination = EquatorialCoordinates.declination;
    }

    rade.rightascension = RightAscension;
    rade.declination = Declination;
    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::saveConfigItems(FILE *fp)
{
    SaveAlignmentConfigProperties(fp);

    IUSaveConfigNumber(fp, &TrackFactorNP);

    return INDI::Telescope::saveConfigItems(fp);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Sync(double ra, double dec)
{
    DEBUG(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "LFAST_Mount::Sync");
    LOG_INFO("LFAST_Mount::Sync");
    double azPosnFb, altPosnFb;
    try
    {
        azPosnFb = AzimuthAxis->getPositionFeedback();
        altPosnFb = AltitudeAxis->getPositionFeedback();
    }
    catch (const std::exception &e)
    {
        LOG_ERROR(e.what());
    }

    // Syncing is treated specially when the telescope position is known in park position to spare
    // "a huge-jump point" in the alignment model.
    if (isParked())
    {
        INDI::IHorizontalCoordinates AltAz{0, 0};
        TelescopeDirectionVector TDV;

        if (TransformCelestialToTelescope(ra, dec, 0.0, TDV))
        {
            AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
            double OrigAlt = AltAz.altitude;
            // ZeroPositionEncoders[AXIS1] = PolarisPositionEncoders[AXIS1] - DegreesToMicrosteps(AXIS1, AltAz.azimuth);
            // ZeroPositionEncoders[AXIS2] = PolarisPositionEncoders[AXIS2] - DegreesToMicrosteps(AXIS2, AltAz.altitude);
            LOGF_INFO("Sync (Alt: %lf Az: %lf) in park position", OrigAlt, AltAz.azimuth);
            GetAlignmentDatabase().clear();
            return true;
        }
    }

    // Might as well do this
    UpdateDetailedMountInformation(true);

    INDI::IHorizontalCoordinates AltAz{0, 0};

    AltAz.azimuth = azPosnFb;
    AltAz.altitude = altPosnFb;

    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Axis1 encoder %ld initial %ld AZ %lf°",
    //        CurrentEncoders[AXIS1], ZeroPositionEncoders[AXIS1], AltAz.azimuth);
    // DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Axis2 encoder %ld initial %ld AL %lf°",
    //        CurrentEncoders[AXIS2], ZeroPositionEncoders[AXIS2], AltAz.altitude);

    AlignmentDatabaseEntry NewEntry;
    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension = ra;
    NewEntry.Declination = dec;
    NewEntry.TelescopeDirection = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
    NewEntry.PrivateDataSize = 0;

    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "New sync point Date %lf RA %lf DEC %lf TDV(x %lf y %lf z %lf)",
           NewEntry.ObservationJulianDate, NewEntry.RightAscension, NewEntry.Declination, NewEntry.TelescopeDirection.x,
           NewEntry.TelescopeDirection.y, NewEntry.TelescopeDirection.z);

    m_IterativeGOTOPending = false;

    if (!CheckForDuplicateSyncPoint(NewEntry))
    {
        GetAlignmentDatabase().push_back(NewEntry);

        // Tell the client about size change
        UpdateSize();

        // Tell the math plugin to reinitialise
        Initialise(this);

        // Force read before restarting
        ReadScopeStatus();

        // The tracking seconds should be reset to restart the drift compensation
        resetTracking();

        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::Abort()
{
    // DEBUG(DBG_SCOPE, "LFAST_Mount::Abort");
    m_IterativeGOTOPending = false;
    AltitudeAxis->slowStop();
    AzimuthAxis->slowStop();
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

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::TimerHit()
{
    // Call parent to read ReadScopeStatus
    INDI::Telescope::TimerHit();

    switch (TrackState)
    {
    case SCOPE_SLEWING:
        GuideDeltaAlt = 0;
        GuideDeltaAz = 0;
        ResetGuidePulses();
        GuidingPulses.clear();
        break;

    case SCOPE_TRACKING:
    {
        // Check if manual motion in progress but we stopped
        if (m_ManualMotionActive && AzimuthAxis->isStopped() && AltitudeAxis->isStopped())
        {
            m_ManualMotionActive = false;
            resetTracking();
        }
        // If we're manually moving by WESN controls, update the tracking coordinates.
        if (m_ManualMotionActive)
        {
            break;
        }
        else
        {

            // Continue or start tracking
            // Calculate where the mount needs to be in POLLMS time
            // TODO may need to make this longer to get a meaningful result
            // double JulianOffset = (getCurrentPollingPeriod() / 1000) / (24.0 * 60 * 60);
            TelescopeDirectionVector TDV;
            INDI::IHorizontalCoordinates AltAz{0, 0};

            if (TransformCelestialToTelescope(m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination,
                                              0, TDV))
            {
                DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);
                AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
            }
            else
            {
                INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
                EquatorialCoordinates.rightascension = m_SkyTrackingTarget.rightascension;
                EquatorialCoordinates.declination = m_SkyTrackingTarget.declination;
                INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &AltAz);
            }
            AltitudeAxis->updateTrackCommands(AltAz.altitude);
            AzimuthAxis->updateTrackCommands(AltAz.azimuth);
            // DEBUGF(DBG_SCOPE,
            //        "Tracking AXIS1 CurrentEncoder %ld OldTrackingTarget %ld AXIS2 CurrentEncoder %ld OldTrackingTarget "
            //        "%ld",
            //        CurrentEncoders[AXIS1],
            //        OldTrackingTarget[AXIS1],
            //        CurrentEncoders[AXIS2],
            //        OldTrackingTarget[AXIS2]);

            // DEBUGF(DBG_SCOPE,
            //        "New Tracking Target AZ %lf° (%ld microsteps) AL %lf° (%ld microsteps) ",
            //        AltAz.azimuth,
            //        DegreesToMicrosteps(AXIS1, AltAz.azimuth),
            //        AltAz.altitude,
            //        DegreesToMicrosteps(AXIS2, AltAz.altitude));

            // Calculate the auto-guiding delta degrees
            double DeltaAlt = 0;
            double DeltaAz = 0;

            for (auto Iter = GuidingPulses.begin(); Iter != GuidingPulses.end();)
            {
                // We treat the guide calibration specially
                if (Iter->OriginalDuration == 1000)
                {
                    DeltaAlt += Iter->DeltaAlt;
                    DeltaAz += Iter->DeltaAz;
                }
                else
                {
                    DeltaAlt += Iter->DeltaAlt / 2;
                    DeltaAz += Iter->DeltaAz / 2;
                }
                Iter->Duration -= getCurrentPollingPeriod();

                if (Iter->Duration < static_cast<int>(getCurrentPollingPeriod()))
                {
                    Iter = GuidingPulses.erase(Iter);
                    if (Iter == GuidingPulses.end())
                    {
                        break;
                    }
                    continue;
                }
                ++Iter;
            }

            GuideDeltaAlt += DeltaAlt;
            GuideDeltaAz += DeltaAz;

            // long AzimuthOffsetMicrosteps = DegreesToMicrosteps(AXIS1,
            //                                                    AltAz.azimuth + GuideDeltaAz) +
            //                                ZeroPositionEncoders[AXIS1] - CurrentEncoders[AXIS1];
            // long AltitudeOffsetMicrosteps = DegreesToMicrosteps(AXIS2,
            //                                                     AltAz.altitude + GuideDeltaAlt) +
            //                                 ZeroPositionEncoders[AXIS2] - CurrentEncoders[AXIS2];

            // DEBUGF(DBG_SCOPE, "New Tracking Target AZOffset %ld microsteps ALOffset %ld microsteps.",
            //        AltitudeOffsetMicrosteps, AzimuthOffsetMicrosteps);

            // Going the long way round - send it the other way
            // if (AzimuthOffsetMicrosteps > MicrostepsPerRevolution[AXIS1] / 2)
            //     AzimuthOffsetMicrosteps -= MicrostepsPerRevolution[AXIS1];

            // if (0 != AzimuthOffsetMicrosteps)
            // {
            //     // Calculate the slewing rates needed to reach that position
            //     // at the correct time.
            //     long AzimuthRate = StepperClockFrequency[AXIS1] / AzimuthOffsetMicrosteps;
            //     if (!AxesStatus[AXIS1].FullStop && ((AxesStatus[AXIS1].SlewingForward && (AzimuthRate < 0)) ||
            //                                         (!AxesStatus[AXIS1].SlewingForward && (AzimuthRate > 0))))
            //     {
            //         // Direction change whilst axis running
            //         // Abandon tracking for this clock tick
            //         DEBUG(DBG_SCOPE, "Tracking -> AXIS1 direction change.");
            //         AzimuthAxis->slowStop();
            //     }
            //     else
            //     {
            //         char Direction = AzimuthRate > 0 ? '0' : '1';
            //         AzimuthRate = std::abs(AzimuthRate) * TrackFactorNP[AXIS_AZ].getValue();
            //         SetClockTicksPerMicrostep(AXIS1, AzimuthRate < 1 ? 1 : AzimuthRate);
            //         if (AxesStatus[AXIS1].FullStop)
            //         {
            //             DEBUG(DBG_SCOPE, "Tracking -> AXIS1 restart.");
            //             SetAxisMotionMode(AXIS1, '1', Direction);
            //             StartAxisMotion(AXIS1);
            //         }
            //         DEBUGF(DBG_SCOPE, "Tracking -> AXIS1 offset %ld microsteps rate %ld direction %c",
            //                AzimuthOffsetMicrosteps, AzimuthRate, Direction);
            //     }
            // }
            // else
            // {
            //     // Nothing to do - stop the axis
            //     DEBUG(DBG_SCOPE, "Tracking -> AXIS1 zero offset.");
            //     slowStop(AXIS1);
            // }

            // // Going the long way round - send it the other way
            // if (AltitudeOffsetMicrosteps > MicrostepsPerRevolution[AXIS2] / 2)
            //     AltitudeOffsetMicrosteps -= MicrostepsPerRevolution[AXIS2];

            // if (0 != AltitudeOffsetMicrosteps)
            // {
            //     // Calculate the slewing rates needed to reach that position
            //     // at the correct time.
            //     long AltitudeRate = StepperClockFrequency[AXIS2] / AltitudeOffsetMicrosteps;

            //     if (!AxesStatus[AXIS2].FullStop && ((AxesStatus[AXIS2].SlewingForward && (AltitudeRate < 0)) ||
            //                                         (!AxesStatus[AXIS2].SlewingForward && (AltitudeRate > 0))))
            //     {
            //         // Direction change whilst axis running
            //         // Abandon tracking for this clock tick
            //         DEBUG(DBG_SCOPE, "Tracking -> AXIS2 direction change.");
            //         slowStop(AXIS2);
            //     }
            //     else
            //     {
            //         char Direction = AltitudeRate > 0 ? '0' : '1';
            //         AltitudeRate = std::abs(AltitudeRate) * TrackFactorNP[AXIS_ALT].getValue();
            //         SetClockTicksPerMicrostep(AXIS2, AltitudeRate < 1 ? 1 : AltitudeRate);
            //         if (AxesStatus[AXIS2].FullStop)
            //         {
            //             DEBUG(DBG_SCOPE, "Tracking -> AXIS2 restart.");
            //             SetAxisMotionMode(AXIS2, '1', Direction);
            //             StartAxisMotion(AXIS2);
            //         }
            //         DEBUGF(DBG_SCOPE, "Tracking -> AXIS2 offset %ld microsteps rate %ld direction %c",
            //                AltitudeOffsetMicrosteps, AltitudeRate, Direction);
            //     }
            // }
            // else
            // {
            //     // Nothing to do - stop the axis
            //     DEBUG(DBG_SCOPE, "Tracking -> AXIS2 zero offset.");
            //     slowStop(AXIS2);
            // }

            // DEBUGF(DBG_SCOPE, "Tracking -> AXIS1 error %d AXIS2 error %d.",
            //        OldTrackingTarget[AXIS1] - CurrentEncoders[AXIS1],
            //        OldTrackingTarget[AXIS2] - CurrentEncoders[AXIS2]);

            // OldTrackingTarget[AXIS1] = AzimuthOffsetMicrosteps + CurrentEncoders[AXIS1];
            // OldTrackingTarget[AXIS2] = AltitudeOffsetMicrosteps + CurrentEncoders[AXIS2];
        }
    }
    break;

    default:
        GuideDeltaAlt = 0;
        GuideDeltaAz = 0;
        ResetGuidePulses();
        GuidingPulses.clear();
        break;
    }
    updateSim();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::updateSim()
{
    static struct timeval ltv
    {
        0, 0
    }; // previous system time
    struct timeval tv
    {
        0, 0
    }; // new system time

    double dt; // Elapsed time in seconds since last tick

    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;
    // LOGF_INFO("dt: %10.8f", dt);
    AltitudeAxis->simulate(dt);
    AzimuthAxis->simulate(dt);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::updateLocation(double latitude, double longitude, double elevation)
{
    // DEBUG(DBG_SCOPE, "LFAST_Mount::updateLocation");
    UpdateLocation(latitude, longitude, elevation);
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
        // Update location if loaded already from config
        if (m_Location.longitude > 0)
            UpdateLocation(m_Location.latitude, m_Location.longitude, m_Location.elevation);

        // Fill in any real values now available MCInit should have been called already
        UpdateDetailedMountInformation(false);

        // Define our connected only properties to the base driver
        // e.g. defineProperty(MyNumberVectorPointer);
        // This will register our properties and send a IDDefXXXX message to any connected clients
        // I have now idea why I have to do this here as well as in ISGetProperties. It makes me
        // concerned there is a design or implementation flaw somewhere.
        // defineProperty(&BasicMountInfoTP);
        // defineProperty(&AxisOneInfoNP);
        defineProperty(&AxisOneStateSP);
        // defineProperty(&AxisTwoInfoNP);
        defineProperty(&AxisTwoStateSP);
        // defineProperty(&AxisOneEncoderValuesNP);
        // defineProperty(&AxisTwoEncoderValuesNP);
        defineProperty(&SlewModesSP);
        defineProperty(&SoftPECModesSP);
        defineProperty(&SoftPecNP);
        defineProperty(&GuidingRatesNP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&TrackFactorNP);
        defineProperty(&AzPosnNP);
        defineProperty(&AltPosnNP);
        defineProperty(&AzRateNP);
        defineProperty(&AltRateNP);
        // if (HasAuxEncoders())
        // {
        //     LOG_WARN("AUX encoders detected. Turning off...");
        //     TurnRAEncoder(false);
        //     TurnDEEncoder(false);
        //     defineProperty(&AUXEncoderSP);
        // }

        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(GetAxis1Park());
            SetAxis2ParkDefault(GetAxis2Park());
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(0x800000);
            SetAxis2Park(0x800000);
            SetAxis1ParkDefault(0x800000);
            SetAxis2ParkDefault(0x800000);
        }

        if (isParked())
        {
            AzimuthAxis->setPosition(default_park_posn_az);
            AltitudeAxis->setPosition(default_park_posn_alt);
        }
        return true;
    }
    else
    {
        // Delete any connected only properties from the base driver's list
        // e.g. deleteProperty(MyNumberVector.name);
        // deleteProperty(BasicMountInfoTP.name);
        // deleteProperty(AxisOneInfoNP.name);
        deleteProperty(AxisOneStateSP.name);
        // deleteProperty(AxisTwoInfoNP.name);
        deleteProperty(AxisTwoStateSP.name);
        // deleteProperty(AxisOneEncoderValuesNP.name);
        // deleteProperty(AxisTwoEncoderValuesNP.name);
        deleteProperty(SlewModesSP.name);
        deleteProperty(SoftPECModesSP.name);
        deleteProperty(SoftPecNP.name);
        deleteProperty(GuidingRatesNP.name);
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(TrackFactorNP.getName());

        // if (HasAuxEncoders())
        //     deleteProperty(AUXEncoderSP.getName());

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideNorth(uint32_t ms)
{
    GuidingPulse Pulse;

    CalculateGuidePulses();
    Pulse.DeltaAz = NorthPulse.DeltaAz;
    Pulse.DeltaAlt = NorthPulse.DeltaAlt;
    Pulse.Duration = ms;
    Pulse.OriginalDuration = ms;
    GuidingPulses.push_back(Pulse);
    return IPS_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideSouth(uint32_t ms)
{
    GuidingPulse Pulse;

    CalculateGuidePulses();
    Pulse.DeltaAz = -NorthPulse.DeltaAz;
    Pulse.DeltaAlt = -NorthPulse.DeltaAlt;
    Pulse.Duration = ms;
    Pulse.OriginalDuration = ms;
    GuidingPulses.push_back(Pulse);
    return IPS_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideWest(uint32_t ms)
{
    GuidingPulse Pulse;

    CalculateGuidePulses();
    Pulse.DeltaAz = WestPulse.DeltaAz;
    Pulse.DeltaAlt = WestPulse.DeltaAlt;
    Pulse.Duration = ms;
    Pulse.OriginalDuration = ms;
    GuidingPulses.push_back(Pulse);
    return IPS_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
IPState LFAST_Mount::GuideEast(uint32_t ms)
{
    GuidingPulse Pulse;

    CalculateGuidePulses();
    Pulse.DeltaAz = -WestPulse.DeltaAz;
    Pulse.DeltaAlt = -WestPulse.DeltaAlt;
    Pulse.Duration = ms;
    Pulse.OriginalDuration = ms;
    GuidingPulses.push_back(Pulse);
    return IPS_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::CalculateGuidePulses()
{
    if (NorthPulse.Duration != 0 || WestPulse.Duration != 0)
        return;

    // Calculate the west reference delta
    // Note: The RA is multiplied by 3.75 (90/24) to be more comparable with DEC values.
    const double WestRate = IUFindNumber(&GuidingRatesNP, "GUIDERA_RATE")->value / 10 * -1.0 / 60 / 60 * 3.75 / 100;

    ConvertGuideCorrection(WestRate, 0, WestPulse.DeltaAlt, WestPulse.DeltaAz);
    WestPulse.Duration = 1;

    // Calculate the north reference delta
    // Note: By some reason, it must be multiplied by 100 to match with the RA values.
    const double NorthRate = IUFindNumber(&GuidingRatesNP, "GUIDEDEC_RATE")->value / 10 * 1.0 / 60 / 60 * 100 / 100;

    ConvertGuideCorrection(0, NorthRate, NorthPulse.DeltaAlt, NorthPulse.DeltaAz);
    NorthPulse.Duration = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::ResetGuidePulses()
{
    NorthPulse.Duration = 0;
    WestPulse.Duration = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::ConvertGuideCorrection(double delta_ra, double delta_dec, double &delta_alt, double &delta_az)
{
    INDI::IHorizontalCoordinates OldAltAz{0, 0};
    INDI::IHorizontalCoordinates NewAltAz{0, 0};
    TelescopeDirectionVector OldTDV;
    TelescopeDirectionVector NewTDV;

    TransformCelestialToTelescope(m_SkyTrackingTarget.rightascension, m_SkyTrackingTarget.declination, 0.0, OldTDV);
    AltitudeAzimuthFromTelescopeDirectionVector(OldTDV, OldAltAz);
    TransformCelestialToTelescope(m_SkyTrackingTarget.rightascension + delta_ra,
                                  m_SkyTrackingTarget.declination + delta_dec, 0.0, NewTDV);
    AltitudeAzimuthFromTelescopeDirectionVector(NewTDV, NewAltAz);
    delta_alt = NewAltAz.altitude - OldAltAz.altitude;
    delta_az = NewAltAz.azimuth - OldAltAz.azimuth;
}
#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::SkywatcherMicrostepsFromTelescopeDirectionVector(
    const TelescopeDirectionVector TelescopeDirectionVector, long &Axis1Microsteps, long &Axis2Microsteps)
{
    // For the time being I assume that all skywathcer mounts share the same encoder conventions
    double Axis1Angle = 0;
    double Axis2Angle = 0;
    SphericalCoordinateFromTelescopeDirectionVector(TelescopeDirectionVector, Axis1Angle,
                                                    TelescopeDirectionVectorSupportFunctions::CLOCKWISE, Axis1Angle,
                                                    FROM_AZIMUTHAL_PLANE);

    Axis1Microsteps = RadiansToMicrosteps(AXIS1, Axis1Angle);
    Axis2Microsteps = RadiansToMicrosteps(AXIS2, Axis2Angle);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
const TelescopeDirectionVector
LFAST_Mount::TelescopeDirectionVectorFromSkywatcherMicrosteps(long Axis1Microsteps, long Axis2Microsteps)
{
    // For the time being I assume that all skywathcer mounts share the same encoder conventions
    double Axis1Angle = MicrostepsToRadians(AXIS1, Axis1Microsteps);
    double Axis2Angle = MicrostepsToRadians(AXIS2, Axis2Microsteps);
    return TelescopeDirectionVectorFromSphericalCoordinate(
        Axis1Angle, TelescopeDirectionVectorSupportFunctions::CLOCKWISE, Axis2Angle, FROM_AZIMUTHAL_PLANE);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::UpdateDetailedMountInformation(bool InformClient)
{

    AzPosnNP[FEEDBACK].setValue(m_MountAltAz.azimuth);
    AzPosnNP[COMMAND].setValue(AzimuthAxis->getPositionCommand());
    AzPosnNP.apply();

    AltPosnNP[FEEDBACK].setValue(m_MountAltAz.altitude);
    AltPosnNP[COMMAND].setValue(AltitudeAxis->getPositionCommand());
    AltPosnNP.apply();

    AzRateNP[COMMAND].setValue(AzimuthAxis->getVelocityCommand());
    AzRateNP[FEEDBACK].setValue(AzimuthAxis->getVelocityFeedback());
    AzRateNP.apply();

    AltRateNP[COMMAND].setValue(AltitudeAxis->getVelocityCommand());
    AltRateNP[FEEDBACK].setValue(AltitudeAxis->getVelocityFeedback());
    AltRateNP.apply();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::SetCurrentPark()
{
    double azPosnFb = AzimuthAxis->getPositionFeedback();
    double altPosnFb = AltitudeAxis->getPositionFeedback();
    SetAxis1Park(azPosnFb);
    SetAxis2Park(altPosnFb);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool LFAST_Mount::SetDefaultPark()
{
    // Zero azimuth looking north/south (depending on hemisphere)
    // double azPosnFb = AzimuthAxis->getPositionFeedback();
    // double altPosnFb = AltitudeAxis->getPositionFeedback();

    // SetAxis1Park(ZeroPositionEncoders[AXIS1]);
    // SetAxis2Park(ZeroPositionEncoders[AXIS2]);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Restart the drift compensation after syncing or after stopping manual motion
//////////////////////////////////////////////////////////////////////////////////////////////////
void LFAST_Mount::resetTracking()
{
    m_TrackingRateTimer.restart();
    GuideDeltaAlt = 0;
    GuideDeltaAz = 0;
    ResetGuidePulses();
}
