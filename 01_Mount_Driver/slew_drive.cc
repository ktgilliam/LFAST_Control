#include "slew_drive.h"

#include <cmath>
#include <sstream>
#include <exception>

#include "lfast_constants.h"
#include "../00_Utils/math_util.h"
#include "../00_Utils/PID_Controller.h"
#include "../00_Utils/KinkoDriver.h"
const unsigned DriveA_ID = 1;
const unsigned DriveB_ID = 2;

/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////
#define MULT 1
constexpr double MAX_RATE_CMD = 0.25 * MULT;
constexpr double MIN_RATE_CMD = -0.25 * MULT;
double FAKE_SLEW_DRIVE_MAX_SPEED_DPS = 500;

namespace lfc = LFAST_CONSTANTS;

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
SlewDrive::SlewDrive(const char *label)
{
    axisLabel = label;
    // Initialize state variables
    isEnabled = false;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommandOffset_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
    combinedRateCmdSaturated_dps = 0.0;

    pid = std::unique_ptr<PID_Controller>(
        new PID_Controller(
            lfc::SLEW_POSN_KP,
            lfc::SLEW_POSN_KI,
            lfc::SLEW_POSN_KD));

#if SIM_MODE_ENABLED
    driveModelPtr = std::unique_ptr<DF2_IIR<double>>(
        new DF2_IIR<double>(
            DIGITAL_CONTROL::lpf_3_b,
            DIGITAL_CONTROL::lpf_3_a,
            2));
#else
    pDriveA = std::unique_ptr<KinkoDriver>(new KinkoDriver(DriveA_ID));
    pDriveB = std::unique_ptr<KinkoDriver>(new KinkoDriver(DriveB_ID));
#endif

    updateSlewRate(MAX_RATE_CMD);
    pid->reset();

    // pid = new PID_Controller(lfc::SLEW_POSN_KP, lfc::SLEW_POSN_KI, lfc::SLEW_POSN_KD);
}

bool SlewDrive::connect(const char *devPath)
{
#if !SIM_MODE_ENABLED
    KinkoDriver::connectRTU(devPath);
    return KinkoDriver::IsConnected();
#else
    return true;
#endif
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateTrackCommands(double pcmd, double rcmd)
{
    positionCommand_deg = pcmd;
    rateCommandOffset_dps = rcmd;
    // delete pid;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::abortSlew()
{
    positionCommand_deg = positionFeedback_deg;
    rateRef_dps = 0.0;
    rateCommandOffset_dps = 0.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::syncPosition(double posn)
{
    rateCommandOffset_dps = 0.0;
    positionCommand_deg = posn;
    positionFeedback_deg = posn;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
bool SlewDrive::isSlewComplete()
{

    double posnError = positionCommand_deg - positionFeedback_deg;
    int errSign = sign(posnError);
    while (std::abs(posnError) > 180.0)
    {
        posnError -= 360.0 * errSign;
    }
    bool isComplete = (std::abs(posnError) <= SLEW_COMPLETE_THRESH_POSN) &&
                      (std::abs(combinedRateCmdSaturated_dps) < SLEW_COMPLETE_THRESH_RATE);
    return isComplete;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateSlewRate(double slewRate)
{
    rateLim = slewRate;
    pid->configureOutputSaturation(-1 * rateLim, rateLim);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateRateOffset(double rate)
{
    rateCommandOffset_dps = rate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::enable()
{
    if (!KinkoDriver::IsConnected())
    {
        throw std::runtime_error("enable:: Drivers not connected.");
    }

    pDriveA->setControlMode(KINKO::MOTOR_MODE_SPEED);
    pDriveB->setControlMode(KINKO::MOTOR_MODE_SPEED);
    pDriveA->setDriverState(KINKO::MOTOR_STATE_ENABLE);
    pDriveB->setDriverState(KINKO::MOTOR_STATE_ENABLE);
    pDriveA->setDriverState(KINKO::MOTOR_STATE_ON);
    pDriveB->setDriverState(KINKO::MOTOR_STATE_ON);
    isEnabled = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::disable()
{
    if (KinkoDriver::IsConnected())
    {
        pDriveA->setDriverState(KINKO::MOTOR_STATE_DISABLE);
        pDriveB->setDriverState(KINKO::MOTOR_STATE_DISABLE);
    }

    isEnabled = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateControlLoops(double dt, ControlMode_t mode)
{

    double posnError = positionCommand_deg - positionFeedback_deg;
    int errSign = sign(posnError);
    while (std::abs(posnError) > 180.0)
    {
        posnError -= 360.0 * errSign;
    }
    if (std::abs(posnError) < lfc::POSN_PID_ENABLE_THRESH_DEG)
    {
        pid->update(posnError, dt, &rateRef_dps);
    }
    else
    {
        rateRef_dps = saturate(posnError, -1 * rateLim, rateLim); // * sign(posnError);
    }

    double combinedRateCmd_dps{0};
    static ControlMode_t prevMode = POSN_CONTROL_ONLY;
    if (mode != prevMode)
    {
        pid->reset();
    }

    if (mode == POSN_CONTROL_ONLY)
    {
        combinedRateCmd_dps = saturate(rateRef_dps, -1 * rateLim, rateLim);
    }
    else if (mode == POSN_AND_RATE_CONTROL)
    {

        combinedRateCmd_dps = saturate(rateRef_dps, -1 * rateLim, rateLim) + rateCommandOffset_dps;
        // combinedRateCmd_dps = saturate(rateCommandOffset_dps, -1 * rateLim, rateLim);
    }

#if SIM_MODE_ENABLED
    combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps,
                                            -1 * FAKE_SLEW_DRIVE_MAX_SPEED_DPS,
                                            FAKE_SLEW_DRIVE_MAX_SPEED_DPS);
#else
    combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps,
                                            -1 * LFAST_CONSTANTS::SLEW_DRIVE_MAX_SPEED_DPS,
                                            LFAST_CONSTANTS::SLEW_DRIVE_MAX_SPEED_DPS);

    double motorVelCommand_RPM = mapSlewDriveCommandToMotors();
    // The worm gears have to turn opposite directions
    if (!KinkoDriver::IsConnected())
    {
        throw std::runtime_error("updateControlLoops:: Drivers not connected.");
    }
    if (isEnabled)
    {
        motorVelCommand_RPM = 200.0*LFAST_CONSTANTS::INV_TOTAL_GEAR_RATIO;
        pDriveA->updateVelocityCommand(motorVelCommand_RPM);
        pDriveB->updateVelocityCommand(-1 * motorVelCommand_RPM);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::mapSlewDriveCommandToMotors()
{
    // TODO: Implement proper state estimation
    double motorSpeed_dps = combinedRateCmdSaturated_dps * LFAST_CONSTANTS::TOTAL_GEAR_RATIO;
    double motorSpeed_rpm = motorSpeed_dps / 6;
    return (motorSpeed_rpm);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::getPositionFeedback()
{
#if SIM_MODE_ENABLED
    return std::fmod(positionFeedback_deg, 360.0);
#else
    if (!KinkoDriver::IsConnected())
    {
        throw std::runtime_error("getPositionFeedback:: Drivers not connected.");
    }

    double drvAPosn = pDriveA->getPositionFeedback();
    double drvBPosn = pDriveA->getPositionFeedback();
    double drvPosnAve = (drvAPosn + drvBPosn) * 0.5;
    positionFeedback_deg = processPositionFeedback(drvPosnAve);
    return positionFeedback_deg;
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::processPositionFeedback(double currPosn)
{
    static double prevPosn;
    static unsigned prevSector;
    unsigned currSector;
    static bool firstTime = true;

    if (firstTime)
    {
        prevPosn = currPosn;
        currSector = 0;
    }

    const unsigned NUM_SECTORS = 10;
    const unsigned MAX_SECTOR = NUM_SECTORS - 1;
    const double SECTOR_SIZE = 360.0 / NUM_SECTORS;

    double deltaPosn = currPosn - prevPosn;
    currSector = (unsigned)currPosn / NUM_SECTORS;

    // bool wrapOccurred = false;
    static int wrapCounter = 0;
    if (currSector == 0 && prevSector == MAX_SECTOR)
    {
        wrapCounter++;
    }
    else if (currSector == MAX_SECTOR && prevSector == 0)
    {
        wrapCounter--;
    }
    else
    {
        int deltaSector = currSector - prevSector;
        if (std::abs(deltaSector) > 1)
        {
            throw std::runtime_error("Encoder unwrap error");
        }
    }

    double outputPosn = (wrapCounter * 360) + currPosn;
    prevPosn = currPosn;
    prevSector = currSector;
    return outputPosn;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::getVelocityFeedback()
{
#if !SIM_MODE_ENABLED
    if (!KinkoDriver::IsConnected())
    {
        throw std::runtime_error("getVelocityFeedback:: Drivers not connected.");
    }
    double drvAVel = pDriveA->getVelocityFeedback();
    double drvBVel = pDriveB->getVelocityFeedback();
    double drvVelAve = (drvAVel - drvBVel) * 0.5;
    rateFeedback_dps = drvVelAve;

#endif
    return rateFeedback_dps;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
#if SIM_MODE_ENABLED

void SlewDrive::simulate(double dt)
{
    rateFeedback_dps = driveModelPtr->update(combinedRateCmdSaturated_dps);
    double deltaPos = rateFeedback_dps * dt;
    positionFeedback_deg += deltaPos;
}

#endif
