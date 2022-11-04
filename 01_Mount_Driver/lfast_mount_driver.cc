/*
   INDI Developers Manual
   Tutorial #7

   "Simple telescope simulator"

   We construct a most basic (and useless) device driver to illustrate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

#include "lfast_mount_driver.h"

#include "indicom.h"
#include <libnova/julian_day.h>
#include <memory>

#include "slew_drive.h"

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
#define SLEWMODES 9
const double slewspeeds[SLEWMODES] = {1.0, 2.0, 4.0, 8.0, 32.0, 64.0, 128.0, 256.0, 512.0};

const double constexpr default_park_posn_az = 0.0;
const double constexpr default_park_posn_alt = -110.0;
const unsigned int defaultPollingPeriod = 100;

// We declare an auto pointer to LFAST_Mount.
std::unique_ptr<LFAST_Mount> lfast_mount(new LFAST_Mount());

LFAST_Mount::LFAST_Mount() : DBG_SIMULATOR(INDI::Logger::getInstance().addDebugLevel("Simulator Verbose", "SIMULATOR"))
{
    // Set up the basic configuration for the mount
    setVersion(0, 3);
    setTelescopeConnection(CONNECTION_TCP);
    SetTelescopeCapability(SCOPE_CAPABILITIES, SLEWMODES);

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    AltitudeAxis = new SlewDrive();
    AzimuthAxis = new SlewDrive();
}

LFAST_Mount::~LFAST_Mount()
{
    delete AltitudeAxis;
    delete AzimuthAxis;
}

const char *LFAST_Mount::getDefaultName()
{
    return (const char *)"LFAST Mount Control";
}

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
        // Axis 1 is AZ
        // Axis 2 is ALT
        SetAxis1ParkDefault(default_park_posn_az);
        SetAxis2ParkDefault(default_park_posn_alt);
    }
    else
    {
        // Otherwise, we set all parking data to default in case no parking data is found.
        SetAxis1Park(default_park_posn_az);
        SetAxis2Park(default_park_posn_alt);
        SetAxis1ParkDefault(default_park_posn_az);
        SetAxis2ParkDefault(default_park_posn_alt);
    }

    setDefaultPollingPeriod(defaultPollingPeriod);
    setCurrentPollingPeriod(defaultPollingPeriod);

    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();

    // Add alignment properties
    InitAlignmentProperties(this);

    // NTP Server Address text field
    NtpServerTP[0].fill("NTP_SERVER_ADDR", "NTP Server", "0.pool.ntp.arizona.edu");
    NtpServerTP.fill(getDeviceName(), "NTP_SERVER_ADDR", "NTP Server", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);
    defineProperty(&NtpServerTP);

    return true;
}

bool LFAST_Mount::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        LOG_WARN("Initial Park status hardcoded");
        SetParked(true);
    }
    else
    {
    }
    return true;
}

bool LFAST_Mount::Connect()
{
    SetTimer(getCurrentPollingPeriod());
    return INDI::Telescope::Connect();
    // return true;
}

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

bool LFAST_Mount::Goto(double ra, double dec)
{

    updateTrackingTarget(ra, dec);

    // Call the alignment subsystem to translate the celestial reference frame coordinate
    // into a telescope reference frame coordinate
    ALIGNMENT::TelescopeDirectionVector TDVCommand;
    INDI::IHorizontalCoordinates AltAzCommand{0, 0};

    if (TransformCelestialToTelescope(ra, dec, 0.0, TDVCommand))
    {
        // The alignment subsystem has successfully transformed my coordinate
        AltitudeAzimuthFromTelescopeDirectionVector(TDVCommand, AltAzCommand);
    }
    else
    {
        // The alignment subsystem cannot transform the coordinate.
        // Try some simple rotations using the stored observatory position if any

        INDI::IEquatorialCoordinates EquatorialCoordinates{ra, dec};
        INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &AltAzCommand);
        TDVCommand = TelescopeDirectionVectorFromAltitudeAzimuth(AltAzCommand);
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
        AltitudeAzimuthFromTelescopeDirectionVector(TDVCommand, AltAzCommand);
    }

    if ((AltAzCommand.altitude > 90.0) || (AltAzCommand.altitude < -90.0))
    {
        DEBUG(DBG_SIMULATOR, "Goto - Altitude out of range");
        // This should not happen
        return false;
    }

    if ((AltAzCommand.azimuth > 360.0) || (AltAzCommand.azimuth < -360.0))
    {
        DEBUG(DBG_SIMULATOR, "Goto - Azimuth out of range");
        // This should not happen
        return false;
    }

    if (AltAzCommand.azimuth < 0.0)
    {
        DEBUG(DBG_SIMULATOR, "Goto - Azimuth negative");
        AltAzCommand.azimuth = 360.0 + AltAzCommand.azimuth;
    }

    DEBUGF(DBG_SIMULATOR, "Goto - Scope reference frame target altitude %lf azimuth %lf", AltAzCommand.altitude,
           AltAzCommand.azimuth);

    // TODO: Try/Catch
    AltitudeAxis->updatePositionCommand(AltAzCommand.altitude);
    AzimuthAxis->updatePositionCommand(AltAzCommand.azimuth);

    TrackState = SCOPE_SLEWING;

    return true;
}

void LFAST_Mount::updateTrackingTarget(double ra, double dec)
{
    DEBUGF(DBG_SIMULATOR, "Goto - Celestial reference frame target right ascension %lf(%lf) declination %lf",
           ra * 360.0 / 24.0, ra, dec);
    if (ISS_ON == IUFindSwitch(&CoordSP, "TRACK")->s)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, ra, 2, 3600);
        fs_sexa(DecStr, dec, 2, 3600);
        CurrentTrackingTarget.rightascension = ra;
        CurrentTrackingTarget.declination = dec;
        DEBUG(DBG_SIMULATOR, "Goto - tracking requested");
    }
}

bool LFAST_Mount::Abort()
{
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

    AltitudeAxis->abortSlew();
    AzimuthAxis->abortSlew();

    AbortSP.s = IPS_OK;
    IUResetSwitch(&AbortSP);
    IDSetSwitch(&AbortSP, nullptr);
    LOG_INFO("Telescope aborted.");

    return true;
}

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

bool LFAST_Mount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentNumberProperties(this, name, values, names, n);
    }

    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

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

bool LFAST_Mount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    LOGF_INFO("MoveNS: dir=%s", getDirString(dir));

    // AxisDirection axisDir = (dir == DIRECTION_NORTH) ? FORWARD : REVERSE;
    // AxisStatus axisStat = (command == MOTION_START) ? SLEWING : STOPPED;

    // AxisSlewRateDEC = DEFAULT_SLEW_RATE;
    // AxisDirectionDEC = axisDir;
    // AxisStatusDEC = axisStat;

    return true;
}

bool LFAST_Mount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    LOGF_INFO("MoveNS: dir=%s", getDirString(dir));

    // AxisDirection axisDir = (dir == DIRECTION_WEST) ? FORWARD : REVERSE;
    // AxisStatus axisStat = (command == MOTION_START) ? SLEWING : STOPPED;

    // AxisSlewRateRA = DEFAULT_SLEW_RATE;
    // AxisDirectionRA = axisDir;
    // AxisStatusRA = axisStat;

    return true;
}

bool LFAST_Mount::Sync(double ra, double dec)
{
    ALIGNMENT::AlignmentDatabaseEntry NewEntry;

    double azFb, altFb;
    azFb = AzimuthAxis->getPositionFeedback();
    altFb = AltitudeAxis->getPositionFeedback();
    INDI::IHorizontalCoordinates AzAltFeedback{azFb, altFb};

    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension = ra;
    NewEntry.Declination = dec;
    NewEntry.TelescopeDirection = TelescopeDirectionVectorFromAltitudeAzimuth(AzAltFeedback);
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
    return false;
}

bool LFAST_Mount::Park()
{
    // INDI::IHorizontalCoordinates AltAzParkingSpot{default_park_posn_az, default_park_posn_alt};
    // INDI::IEquatorialCoordinates EquatorialCoordinates;
    // INDI::HorizontalToEquatorial(&AltAzParkingSpot, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
    // char RAStr[64], DecStr[64];
    // fs_sexa(RAStr, EquatorialCoordinates.rightascension, 2, 3600);
    // fs_sexa(DecStr, EquatorialCoordinates.declination, 2, 3600);
    // LOGF_INFO("Parked RA: %s Parked DEC: %s", RAStr, DecStr);
    AltitudeAxis->gotoAndStop(default_park_posn_alt);
    AzimuthAxis->gotoAndStop(default_park_posn_az);

    // NewRaDec(EquatorialCoordinates.rightascension, EquatorialCoordinates.declination);
    TrackState = SCOPE_PARKING;

    LOG_WARN("Park routine not implemented.");
    // TrackState = SCOPE_PARKED;

    SetParked(true);
    return true;
}
bool LFAST_Mount::UnPark()
{
    LOG_WARN("Unpark button not implemented.");
    SetParked(false);
    TrackState = SCOPE_IDLE;
    return true;
}

bool LFAST_Mount::updateLocation(double latitude, double longitude, double elevation)
{
    UpdateLocation(latitude, longitude, elevation);
    return true;
}

bool LFAST_Mount::ReadScopeStatus()
{
    double azFb, altFb;
    azFb = AzimuthAxis->getPositionFeedback();
    altFb = AltitudeAxis->getPositionFeedback();
    INDI::IHorizontalCoordinates AzAltFeedback{azFb, altFb};

    ALIGNMENT::TelescopeDirectionVector TDV = TelescopeDirectionVectorFromAltitudeAzimuth(AzAltFeedback);
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
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, AzAltFeedback);
            break;

        case ALIGNMENT::SOUTH_CELESTIAL_POLE:
            if (TraceThisTick)
                DEBUG(DBG_SIMULATOR, "ReadScopeStatus - ApproximateMountAlignment SOUTH_CELESTIAL_POLE");
            // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 plus
            // the (negative)observatory latitude. The vector itself is rotated anticlockwise
            RotatedTDV.RotateAroundY(-90.0 - m_Location.latitude);
            AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, AzAltFeedback);
            break;
        }

        INDI::IEquatorialCoordinates EquatorialCoordinates;
        INDI::HorizontalToEquatorial(&AzAltFeedback, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
        // libnova works in decimal degrees
        RightAscension = EquatorialCoordinates.rightascension;
        Declination = EquatorialCoordinates.declination;
    }

    if (TraceThisTick)
        DEBUGF(DBG_SIMULATOR, "ReadScopeStatus - RA %lf hours DEC %lf degrees", RightAscension, Declination);

    NewRaDec(RightAscension, Declination);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
////////////////////////////////////////////////////////////////////////////////////////////?
void LFAST_Mount::TimerHit()
{
    TraceThisTickCount++;
    if (60 == TraceThisTickCount)
    {
        TraceThisTick = true;
        TraceThisTickCount = 0;
    }
    // Simulate mount movement

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

    //     if (TransformCelestialToTelescope(CurrentTrackingTarget.rightascension, CurrentTrackingTarget.declination, JulianOffset,
    //                                       TDV))
    //         AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
    //     else
    //     {

    //         INDI::IEquatorialCoordinates EquatorialCoordinates{0, 0};
    //         EquatorialCoordinates.rightascension = CurrentTrackingTarget.rightascension;
    //         EquatorialCoordinates.declination = CurrentTrackingTarget.declination;
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
void LFAST_Mount::mountSim()
{
}
