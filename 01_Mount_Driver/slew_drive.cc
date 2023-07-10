#include "slew_drive.h"

#include <cmath>
#include <sstream>
#include <exception>

#include "lfast_constants.h"
#include "../00_Utils/math_util.h"
#include "../00_Utils/PID_Controller.h"
#include "../00_Utils/KincoDriver.h"

/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////
#define MULT 1
constexpr double MAX_RATE_CMD = 0.25 * MULT;
constexpr double MIN_RATE_CMD = -0.25 * MULT;
double FAKE_SLEW_DRIVE_MAX_SPEED_DPS = 500;

// namespace lfc = LFAST_CONSTANTS;

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
    positionOffset_deg = 0.0;
    rateCommandFeedforward_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
    combinedRateCmdSaturated_dps = 0.0;
    homingRoutineStatus = HOMING_IDLE;
    // if (simModeEnabled)
    // {
    pid = std::unique_ptr<PID_Controller>(
        new PID_Controller(
            SLEWDRIVE::SLEW_POSN_KP,
            SLEWDRIVE::SLEW_POSN_KI,
            SLEWDRIVE::SLEW_POSN_KD));
    // }

    driveModelPtr = std::unique_ptr<DF2_IIR<double>>(
        new DF2_IIR<double>(
            DIGITAL_CONTROL::lpf_3_b,
            DIGITAL_CONTROL::lpf_3_a,
            2));

    pDriveA = std::unique_ptr<KincoDriver>(new KincoDriver(DriveA_ID));
    pDriveB = std::unique_ptr<KincoDriver>(new KincoDriver(DriveB_ID));

    updateSlewRate(MAX_RATE_CMD);
    pid->reset();

    // pid = new PID_Controller(lfc::SLEW_POSN_KP, lfc::SLEW_POSN_KI, lfc::SLEW_POSN_KD);
}

bool SlewDrive::initializeDriverBus(const char *devPath)
{
    KincoDriver::initializeRTU(devPath);
    return KincoDriver::rtuIsActive();
}

bool SlewDrive::connectToDrivers()
{
    bool result;
    if (!simModeEnabled)
    {
        try
        {
            drvAConnected = pDriveA->driverHandshake();
            drvBConnected = pDriveB->driverHandshake();
            result = drvAConnected && drvBConnected;
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
    positionFeedback_deg = 0.0;
    positionCommand_deg = 0.0;
    rateCommandFeedforward_dps = 0.0;
    rateFeedback_dps = 0.0;
    rateRef_dps = 0.0;
    combinedRateCmdSaturated_dps = 0.0;
    if (!simModeEnabled)
    {
        try
        {
            pDriveA->setDirectionMode(KINCO::CW_IS_POSITIVE);
            pDriveB->setDirectionMode(KINCO::CCW_IS_POSITIVE);

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
    rateCommandFeedforward_dps = rcmd;
    // delete pid;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::abortSlew()
{
    positionCommand_deg = positionFeedback_deg;
    rateRef_dps = 0.0;
    rateCommandFeedforward_dps = 0.0;
    try
    {
        pDriveA->setDriverState(KINCO::ESTOP_VOLTAGE_OFF);
        pDriveB->setDriverState(KINCO::ESTOP_VOLTAGE_OFF);
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
    manualRateCommand_dps = 0.0;
    combinedRateCmdSaturated_dps = 0.0;
    rateCommandFeedforward_dps = 0.0;
    rateRef_dps = 0.0;
    if (!drvAConnected || !drvBConnected)
        return;

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
    else
    {

    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::syncPosition(double sync_posn)
{
    pid->resetIntegrator();
    rateCommandFeedforward_dps = 0.0;
    positionOffset_deg = 0.0;
    positionOffset_deg = sync_posn - getPositionFeedback();
    positionCommand_deg = getPositionFeedback();
    // positionFeedback_deg = sync_posn;
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
    rateCommandFeedforward_dps = rate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::updateManualRateCommand(double rate)
{
    manualRateCommand_dps = rate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::enable()
{
    if (!simModeEnabled)
    {
        if (!KincoDriver::rtuIsActive())
        {
            throw std::runtime_error("enable:: Drivers not connected.");
        }
        try
        {
            pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
            pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
            pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
            pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
            pDriveA->setDriverState(KINCO::POWER_ON_MOTOR);
            pDriveB->setDriverState(KINCO::POWER_ON_MOTOR);
            pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
            pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
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
        if (KincoDriver::rtuIsActive())
        {
            try
            {
                pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
                pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
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
    if (homingRoutineStatus != HOMING_IDLE)
    {
        throw std::runtime_error("updateControlLoops called while homing");
    }
    double posnError = positionCommand_deg - positionFeedback_deg;
    int errSign = sign(posnError);
    while (std::abs(posnError) > 180.0)
    {
        posnError -= 360.0 * errSign;
    }
    if (std::abs(posnError) < SLEWDRIVE::POSN_PID_ENABLE_THRESH_DEG)
    {
        pid->update(posnError, dt, &rateRef_dps);
    }
    else
    {
        rateRef_dps = saturate(posnError, -1 * rateLim, rateLim); // * sign(posnError);
    }

    double combinedRateCmd_dps{0};
    static ControlMode_t prevMode = SLEWING_TO_POSN;
    if (mode != prevMode)
    {
        pid->reset();
    }

    if (mode == SLEWING_TO_POSN)
    {
        combinedRateCmd_dps = saturate(rateRef_dps, -1 * rateLim, rateLim);
    }
    else if (mode == MANUAL_SLEW)
    {
        combinedRateCmd_dps = manualRateCommand_dps; //saturate(manualRateCommand_dps, -1 * rateLim, rateLim);
    }
    else if (mode == TRACKING_COMMAND)
    {
        combinedRateCmd_dps = saturate(rateRef_dps, -1 * rateLim, rateLim) + rateCommandFeedforward_dps;
    }

#if SIM_MODE_ENABLED
    combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps,
                                            -1 * FAKE_SLEW_DRIVE_MAX_SPEED_DPS,
                                            FAKE_SLEW_DRIVE_MAX_SPEED_DPS);
#else
    combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps,
                                            -1 * SLEWDRIVE::SLEW_DRIVE_MAX_SPEED_DPS,
                                            SLEWDRIVE::SLEW_DRIVE_MAX_SPEED_DPS);

    double motorVelCommand_RPM = mapSlewDriveCommandToMotors(combinedRateCmdSaturated_dps);

    if (!simModeEnabled)
    {
        if (!KincoDriver::rtuIsActive())
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
    double motorSpeed_dps = slewRateCmd_dps * SLEWDRIVE::TOTAL_GEAR_RATIO;
    double motorSpeed_rpm = motorSpeed_dps / 6;
    return (motorSpeed_rpm);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double SlewDrive::mapMotorPositionToSlewDrive(double drvPosnAve)
{
    // TODO: Implement proper state estimation
    double motorPosn_deg_tmp = drvPosnAve * SLEWDRIVE::INV_TOTAL_GEAR_RATIO;
    double motorPosn_deg_offs = motorPosn_deg_tmp + positionOffset_deg;
    return motorPosn_deg_offs;
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
        if (!KincoDriver::rtuIsActive())
        {
            throw std::runtime_error("getPositionFeedback:: Drivers not connected.");
        }
        double drvAPosn, drvBPosn;
        try
        {
            // Rename these
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
        if (homingRoutineStatus == HOMING_IDLE)
        {
            // if (std::abs(drvAPosn - drvBPosn) > SLEWDRIVE::MOTOR_MISMATCH_ERROR_THRESH)
            // {
            //     disable();
            //     char errbuff[100];
            //     sprintf(errbuff, "Motor feedbacks not sync'd: Ax: %s, A=%6.4f, B=%6.4f", axisLabel, drvAPosn, drvBPosn);
            //     throw std::runtime_error(errbuff);
            // }
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
    // const double SECTOR_SIZE = 360.0 / NUM_SECTORS;

    // double deltaPosn = currPosn - prevPosn;
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
        if (!KincoDriver::rtuIsActive())
        {
            throw std::runtime_error("getVelocityFeedback:: Drivers not connected.");
        }

        double drvAVel_rpm, drvBVel_rpm, drvVelAve_dps;
        try
        {
            drvAVel_rpm = pDriveA->getVelocityFeedback();
            drvBVel_rpm = pDriveB->getVelocityFeedback();
            drvVelAve_dps = RPM2degpersec((drvAVel_rpm + drvBVel_rpm) * 0.5);
            if (homingRoutineStatus == HOMING_IDLE)
            {
                if (std::abs(drvVelAve_dps) > KINCO::MOTOR_MAX_SPEED_DPS)
                {
                    disable();
                    std::stringstream ss;
                    ss << "Motor drive velocities out of range: A=" << drvAVel_rpm << ", B=" << drvBVel_rpm << ".\n";
                    throw std::runtime_error(ss.str().c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            std::stringstream ss;
            ss << "SlewDrive::getVelocityFeedback() Error [" << axisLabel << "]\n"
               << e.what();
            throw std::runtime_error(ss.str().c_str());
        }

        rateFeedback_dps = drvVelAve_dps * SLEWDRIVE::INV_TOTAL_GEAR_RATIO;
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

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void SlewDrive::startHoming()
{
    if (homingRoutineStatus == HOMING_IDLE)
    {
        homingRoutineStatus = HOME_COMMAND_RECEIVED;
        initializeStates();
    }
    else
    {
        throw std::runtime_error("Homing already in progress");
    }
}

bool SlewDrive::updateAlignment()
{
    bool done = false;

    switch (alignmentCounter++)
    {
    case SLEWDRIVE::ALIGNMENT_STEP_0_START:
        pDriveA->setControlMode(KINCO::MOTOR_MODE_TORQUE);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
        pDriveA->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        pDriveB->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        break;
    case SLEWDRIVE::ALIGNMENT_STEP_1_START:
        pDriveA->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_TORQUE_HARD_BACK);
        pDriveB->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_TORQUE_HARD_BACK);
        break;
    case SLEWDRIVE::ALIGNMENT_STEP_2_START:
        pDriveA->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        pDriveB->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        break;
    case SLEWDRIVE::ALIGNMENT_STEP_3_START:
        pDriveA->setMaxSpeed(degpersec2RPM(180));
        pDriveB->setMaxSpeed(degpersec2RPM(180));
        pDriveA->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_TORQUE_SOFT_FORWARD);
        pDriveB->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_TORQUE_SOFT_FORWARD);
        break;
    case SLEWDRIVE::ALIGNMENT_STEP_4_START:
        pDriveA->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        pDriveB->updateTorqueCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        break;
    case SLEWDRIVE::ALIGNMENT_COMPLETE:
        initializeStates();
        // for some reason changing them back causes them to move even if vcmd is zero
        pDriveA->updateVelocityCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        pDriveB->updateVelocityCommand(SLEWDRIVE::ALIGNMENT_ZERO_TORQUE_SPEED);
        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        done = true;
        break;
    default:
        // do nothing
        break;
    }

    return done;
}

bool SlewDrive::prepForHoming()
{
    bool done = false;
    double drvAVel, drvBVel;

    drvAVel = RPM2degpersec(pDriveA->getVelocityFeedback());
    drvBVel = RPM2degpersec(pDriveB->getVelocityFeedback());

    bool aStopped = std::abs(drvAVel) < 1.0;
    bool bStopped = std::abs(drvBVel) < 1.0;

    if (aStopped && bStopped)
    {

        // Send homing command to drivers
        done = true;
    }

    return done;
}

void SlewDrive::serviceHomingRoutine()
{
    if (simModeEnabled)
    {
        homingRoutineStatus = HOMING_COMPLETE;
    }
    else
    {
        bool alignmentDone, homeCommandSent;
        switch (homingRoutineStatus)
        {
        case HOMING_IDLE:
            break;
        case HOME_COMMAND_RECEIVED:
            alignmentCounter = 0;
            homingRoutineStatus = PRE_HOME_ALIGNMENT;
            break;
        case PRE_HOME_ALIGNMENT:
            alignmentDone = updateAlignment();
            if (alignmentDone)
                homingRoutineStatus = PREPARE_FOR_HOMING;
            break;
        case PREPARE_FOR_HOMING:
            homeCommandSent = prepForHoming();
            if (homeCommandSent)
                homingRoutineStatus = HOMING_ACTIVE;
            break;
        case HOMING_ACTIVE:
            // TODO
            homingRoutineStatus = HOMING_CLEANUP;
            break;
        case HOMING_CLEANUP:
            disable();
            initializeStates();
            homingRoutineStatus = HOMING_COMPLETE;
            break;
        case HOMING_COMPLETE:

            break;
        }
    }
}

bool SlewDrive::isHomingComplete()
{
    if (simModeEnabled)
    {
        return true;
    }
    return (homingRoutineStatus == HOMING_COMPLETE);
}

void SlewDrive::resetHomingRoutine()
{
    homingRoutineStatus = HOMING_IDLE;
}