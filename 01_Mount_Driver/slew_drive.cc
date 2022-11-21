#include "slew_drive.h"

#include <cmath>
#include <sstream>
#include <exception>

#include "lfast_constants.h"
#include "../00_Utils/math_util.h"
#include "../00_Utils/PID_Controller.h"
#include "../00_Utils/KinkoDriver.h"

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
SlewDrive::SlewDrive(const char *label, unsigned DriveA_ID, unsigned DriveB_ID, bool simMode)
{
    axisLabel = label;
    // Initialize state variables
    isEnabled = false;
    simModeEnabled = simMode;

    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommandOffset_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
    combinedRateCmdSaturated_dps = 0.0;

    // if (simModeEnabled)
    // {
    pid = std::unique_ptr<PID_Controller>(
        new PID_Controller(
            lfc::SLEW_POSN_KP,
            lfc::SLEW_POSN_KI,
            lfc::SLEW_POSN_KD));
    // }

    driveModelPtr = std::unique_ptr<DF2_IIR<double>>(
        new DF2_IIR<double>(
            DIGITAL_CONTROL::lpf_3_b,
            DIGITAL_CONTROL::lpf_3_a,
            2));

    pDriveA = std::unique_ptr<KinkoDriver>(new KinkoDriver(DriveA_ID));
    pDriveB = std::unique_ptr<KinkoDriver>(new KinkoDriver(DriveB_ID));

    updateSlewRate(MAX_RATE_CMD);
    pid->reset();

    // pid = new PID_Controller(lfc::SLEW_POSN_KP, lfc::SLEW_POSN_KI, lfc::SLEW_POSN_KD);
}

bool SlewDrive::initializeDriverBus(const char *devPath)
{
    KinkoDriver::initializeRTU(devPath);
    return KinkoDriver::rtuIsActive();
}

bool SlewDrive::connectToDrivers()
{
    bool result;
    if (!simModeEnabled)
    {
        try
        {
            result = pDriveA->driverHandshake();
            result &= pDriveB->driverHandshake();
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::connectToDrivers() Error.\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }
    }
    else
    {
        result = true;
    }

    return result;
}

void SlewDrive::initializeStates()
{
    if (!simModeEnabled)
    {
        try
        {
            pDriveA->setDirectionMode(KINKO::CCW_IS_POSITIVE);
            pDriveB->setDirectionMode(KINKO::CW_IS_POSITIVE);

            pDriveA->zeroPositionOffset();
            pDriveB->zeroPositionOffset();
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::initializeStates() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }
    }
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
    try
    {
        pDriveA->setDriverState(KINKO::MOTOR_STATE_ESTOP);
        pDriveB->setDriverState(KINKO::MOTOR_STATE_ESTOP);
    }
    catch (const std::exception &e)
    {
        std::stringstream ss;
        ss << "SlewDrive::abortSlew() Error [" << axisLabel << "]\n"
           << e.what();
        throw std::runtime_error(ss.str().c_str());
    }
}
void SlewDrive::slowStop()
{
    if (!simModeEnabled)
    {
        try
        {
            pDriveA->updateVelocityCommand(0.0);
            pDriveB->updateVelocityCommand(0.0);
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::slowStop() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }
    }
    // abortSlew();
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
    if (!simModeEnabled)
    {
        if (!KinkoDriver::rtuIsActive())
        {
            throw std::runtime_error("enable:: Drivers not connected.");
        }
        try
        {
            pDriveA->setDriverState(KINKO::MOTOR_STATE_ENABLE);
            pDriveB->setDriverState(KINKO::MOTOR_STATE_ENABLE);
            pDriveA->setDriverState(KINKO::MOTOR_STATE_ON);
            pDriveB->setDriverState(KINKO::MOTOR_STATE_ON);
            pDriveA->setControlMode(KINKO::MOTOR_MODE_SPEED);
            pDriveB->setControlMode(KINKO::MOTOR_MODE_SPEED);
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::enable() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }
    }
    isEnabled = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::disable()
{
    if (!simModeEnabled)
    {
        if (KinkoDriver::rtuIsActive())
        {
            try
            {
                pDriveA->setDriverState(KINKO::MOTOR_STATE_DISABLE);
                pDriveB->setDriverState(KINKO::MOTOR_STATE_DISABLE);
            }
            catch (const std::exception &e)
            {
                std::stringstream ss;
                ss << "SlewDrive::disable() Error [" << axisLabel << "]\n"
                   << e.what();
                throw std::runtime_error(ss.str().c_str());
            }
        }
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

    double motorVelCommand_RPM = mapSlewDriveCommandToMotors(combinedRateCmdSaturated_dps);

    if (!simModeEnabled)
    {
        if (!KinkoDriver::rtuIsActive())
        {
            throw std::runtime_error("updateControlLoops:: Drivers not connected.");
        }
        if (isEnabled)
        {
            try
            {
                pDriveA->updateVelocityCommand(motorVelCommand_RPM);
                pDriveB->updateVelocityCommand(motorVelCommand_RPM);
            }
            catch (const std::exception &e)
            {
                std::stringstream ss;
                ss << "SlewDrive::updateControlLoops() Error [" << axisLabel << "]\n"
                   << e.what();
                throw std::runtime_error(ss.str().c_str());
            }
        }
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::mapSlewDriveCommandToMotors(double slewRateCmd_dps)
{
    // TODO: Implement proper state estimation
    double motorSpeed_dps = slewRateCmd_dps * LFAST_CONSTANTS::TOTAL_GEAR_RATIO;
    double motorSpeed_rpm = motorSpeed_dps / 6;
    return (motorSpeed_rpm);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::mapMotorPositionToSlewDrive(double motorPosn_deg)
{
    // TODO: Implement proper state estimation
    return (motorPosn_deg * LFAST_CONSTANTS::INV_TOTAL_GEAR_RATIO);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::getPositionFeedback()
{
    if (simModeEnabled)
    {
        return std::fmod(positionFeedback_deg, 360.0);
    }
    else
    {
        if (!KinkoDriver::rtuIsActive())
        {
            throw std::runtime_error("getPositionFeedback:: Drivers not connected.");
        }
        double drvAPosn, drvBPosn;
        try
        {
            drvAPosn = pDriveA->getPositionFeedback();
            drvBPosn = pDriveB->getPositionFeedback();
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::getPositionFeedback() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }

        double drvPosnAve = (drvAPosn + drvBPosn) * 0.5;
        // Check difference between them?
        if ((drvAPosn - drvBPosn) > 1.5)
        {
            disable();
            char errbuff[100];
            sprintf(errbuff, "Motor feedbacks not sync'd: A=%6.4f, B=%6.4f", drvAPosn, drvBPosn);
            throw std::runtime_error(errbuff);
        }
        // positionFeedback_deg = processPositionFeedback(drvPosnAve);
        positionFeedback_deg = mapMotorPositionToSlewDrive(drvPosnAve);
        return positionFeedback_deg;
    }
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
        prevSector = 0;
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
            char errBuff[100];
            sprintf(errBuff, "Encoder unwrap error [%d->%d][%6.4f]", currSector, prevSector, currPosn);
            throw std::runtime_error(errBuff);
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
    if (simModeEnabled)
    {
        rateFeedback_dps = driveModelPtr->update(combinedRateCmdSaturated_dps);
    }
    else
    {
        if (!KinkoDriver::rtuIsActive())
        {
            throw std::runtime_error("getVelocityFeedback:: Drivers not connected.");
        }

        double drvAVel, drvBVel;
        try
        {
            drvAVel = pDriveA->getVelocityFeedback();
            drvBVel = pDriveB->getVelocityFeedback();
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::getVelocityFeedback() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }

        double drvVelAve = (drvAVel - drvBVel) * 0.5;
        rateFeedback_dps = drvVelAve;
    }
    return rateFeedback_dps;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::simulate(double dt)
{
    if (simModeEnabled)
    {
        double deltaPos = rateFeedback_dps * dt;
        positionFeedback_deg += deltaPos;
    }
    else
    {
        throw std::runtime_error("simulate() called on non-simulated axis");
    }
}
