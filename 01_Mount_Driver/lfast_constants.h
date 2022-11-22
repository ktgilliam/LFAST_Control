#include <cinttypes>
#include <cmath>
// #include <map>
// #include "math_util.h"
// #include "libastro.h"
#include "../00_Utils/KinkoNamespace.h"
namespace LFAST_CONSTANTS
{
#define EARTH_ROTATIONS_PER_UT1_DAY 1.002737811906
#define SECONDS_PER_DAY 86400.0

    constexpr double SiderealRate_radpersec = (2 * M_PI / SECONDS_PER_DAY) * EARTH_ROTATIONS_PER_UT1_DAY;
    // constexpr double SiderealRate_degpersec = SiderealRate_radpersec * 180.0 / M_PI;
    constexpr double SiderealRate_degpersec = (360.0 / SECONDS_PER_DAY) * EARTH_ROTATIONS_PER_UT1_DAY;
    // const double SiderealRate_radpersec = (0.000072921); //(15.041067 / 3600.0 * M_PI / 180.0)
    // #define SIDEREAL_RATE_DPS 0.004166667

    const double slewspeeds[] = {16.0, 32.0, 64.0, 128.0, 256.0, 512.0};
    constexpr unsigned int NUM_SLEW_SPEEDS = sizeof(slewspeeds) / sizeof(double);
    constexpr unsigned int DEFAULT_SLEW_IDX = NUM_SLEW_SPEEDS - 1;

    enum MOTOR_IDS
    {
        BROADCAST = 0,
        ALTITUDE_MOTOR_A_ID = 1,
        ALTITUDE_MOTOR_B_ID = 2,
        AZIMUTH_MOTOR_A_ID = 3,
        AZIMUTH_MOTOR_B_ID = 4,
    };

}

namespace SLEWDRIVE
{


    const double SLEW_POSN_KP = 0.8;
    const double SLEW_POSN_KI = 0.0;
    const double SLEW_POSN_KD = 0.0;

    const double POSN_PID_ENABLE_THRESH_DEG = 1.5;

    const unsigned int GEAR_BOX_RATIO = 60;
    const unsigned int SLEW_DRIVE_RATIO = 100;
    constexpr unsigned int TOTAL_GEAR_RATIO = GEAR_BOX_RATIO * SLEW_DRIVE_RATIO;
    constexpr double INV_TOTAL_GEAR_RATIO = 1.0 / (double)TOTAL_GEAR_RATIO;

    constexpr double DERATE_RATIO = 0.5;

    constexpr double SLEW_DRIVE_MAX_SPEED_DPS = KINKO::MOTOR_MAX_SPEED_DPS * INV_TOTAL_GEAR_RATIO * DERATE_RATIO;

    constexpr double slewGearBacklash_deg = 1.5;
    constexpr double inputGearBacklash_deg = 0.1;

    constexpr double total_backlash_deg = (inputGearBacklash_deg * GEAR_BOX_RATIO) + (slewGearBacklash_deg * SLEW_DRIVE_RATIO);
    constexpr double MOTOR_MISMATCH_ERROR_THRESH = total_backlash_deg;

    constexpr double max_slew_multiplier = SLEW_DRIVE_MAX_SPEED_DPS / LFAST_CONSTANTS::SiderealRate_degpersec;

    ///////////////////////////////////////////////////////////
    /// SLEW ALIGNMENT ROUTINE STEP DEFINITIONS 
    ///////////////////////////////////////////////////////////
    const unsigned ALIGNMENT_STEP_0_CYCLES = 1;  // Initialize alignment routine
    const unsigned ALIGNMENT_STEP_1_CYCLES = 10;  // Torque hard backwards
    const unsigned ALIGNMENT_STEP_2_CYCLES = 10; // Stop
    const unsigned ALIGNMENT_STEP_3_CYCLES = 20; // Torque soft forwards
    const unsigned ALIGNMENT_STEP_4_CYCLES = 10; // Stop

    const double ALIGNMENT_ZERO_TORQUE_SPEED = 0.0;
    const double ALIGNMENT_TORQUE_HARD_BACK = 1.0;
    const double ALIGNMENT_TORQUE_SOFT_FORWARD = -0.5; 

    constexpr unsigned ALIGNMENT_STEP_0_START = 0;
    constexpr unsigned ALIGNMENT_STEP_1_START = ALIGNMENT_STEP_0_START + ALIGNMENT_STEP_0_CYCLES;
    constexpr unsigned ALIGNMENT_STEP_2_START = ALIGNMENT_STEP_1_START + ALIGNMENT_STEP_1_CYCLES;
    constexpr unsigned ALIGNMENT_STEP_3_START = ALIGNMENT_STEP_2_START + ALIGNMENT_STEP_2_CYCLES;
    constexpr unsigned ALIGNMENT_STEP_4_START = ALIGNMENT_STEP_3_START + ALIGNMENT_STEP_3_CYCLES;
    constexpr unsigned ALIGNMENT_COMPLETE = ALIGNMENT_STEP_4_START + ALIGNMENT_STEP_4_CYCLES;

    ///////////////////////////////////////////////////////////
    /// MOTOR HOMING ROUTINE STEP DEFINITIONS 
    ///////////////////////////////////////////////////////////

}
// INDI::IHorizontalCoordinates HorizontalRates_geocentric(double ha, double dec, double lat);