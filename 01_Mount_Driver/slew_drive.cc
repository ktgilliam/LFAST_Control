#include "slew_drive.h"

#include <cmath>
#include <sstream>

#include "lfast_constants.h"
#include "../00_Utils/math_util.h"
#include "../00_Utils/PID_Controller.h"

/////////////////////////////////////////////////////////////////////////
////////////////////// PUBLIC MEMBER FUNCTIONS //////////////////////////
/////////////////////////////////////////////////////////////////////////
#define MULT 1
constexpr double MAX_RATE_CMD = 0.25 * MULT;
constexpr double MIN_RATE_CMD = -0.25 * MULT;
double SLEW_DRIVE_MAX_SPEED_DPS = 500;

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
#endif

    updateSlewRate(MAX_RATE_CMD);
    pid->reset();

    // pid = new PID_Controller(lfc::SLEW_POSN_KP, lfc::SLEW_POSN_KI, lfc::SLEW_POSN_KD);
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
    bool isComplete = (std::abs(posnError) <= SLEW_COMPLETE_THRESH_POSN ) &&
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
void SlewDrive::enable()
{
    isEnabled = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////

void SlewDrive::updateControl(double dt, ControlMode_t mode)
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

    combinedRateCmdSaturated_dps = saturate(combinedRateCmd_dps, -1 * SLEW_DRIVE_MAX_SPEED_DPS, SLEW_DRIVE_MAX_SPEED_DPS);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
#if SIM_MODE_ENABLED

void SlewDrive::simulate(double dt)
{
    // static double prevRateCmd;
    // static bool firstTime = true;
    // if (firstTime)
    // {
    //     prevRateCmd = combinedRateCmdSaturated_dps;
    //     firstTime = false;
    // }

    rateFeedback_dps = driveModelPtr->update(combinedRateCmdSaturated_dps);
    // prevRateCmd = combinedRateCmdSaturated_dps;
    double deltaPos = rateFeedback_dps * dt;

    // if ((std::abs(deltaPos) <= SLEW_COMPLETE_THRESH) && std::abs(rateCommandOffset_dps) == 0.0)
    // {
    //     positionFeedback_deg = positionCommand_deg;
    //     rateCommandOffset_dps = 0;
    //     rateRef_dps = 0.0;
    //     rateFeedback_dps = 0.0;
    // }
    // else
    // {
    //     positionFeedback_deg += deltaPos;
    // }

    positionFeedback_deg += deltaPos;
}

#endif
