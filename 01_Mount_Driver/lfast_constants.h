#include <cinttypes>
#include <cmath>
// #include <map>
// #include "math_util.h"
// #include "libastro.h"

namespace LFAST_CONSTANTS
{
#define EARTH_ROTATIONS_PER_UT1_DAY 1.002737811906
#define SECONDS_PER_DAY 86400.0

    constexpr double SiderealRate_radpersec = (2 * M_PI / SECONDS_PER_DAY) * EARTH_ROTATIONS_PER_UT1_DAY;
    // constexpr double SiderealRate_degpersec = SiderealRate_radpersec * 180.0 / M_PI;
    constexpr double SiderealRate_degpersec = (360.0 / SECONDS_PER_DAY) * EARTH_ROTATIONS_PER_UT1_DAY;
    // const double SiderealRate_radpersec = (0.000072921); //(15.041067 / 3600.0 * M_PI / 180.0)
    // #define SIDEREAL_RATE_DPS 0.004166667

    const double SLEW_POSN_KP = 0.8;
    const double SLEW_POSN_KI = 0.0;
    const double SLEW_POSN_KD = 0.0;

    const double POSN_PID_ENABLE_THRESH_DEG = 1.5;

    const unsigned int GEAR_BOX_RATIO = 60;
    const unsigned int SLEW_DRIVE_RATIO = 100;
    constexpr unsigned int TOTAL_GEAR_RATIO = GEAR_BOX_RATIO * SLEW_DRIVE_RATIO;
    constexpr double INV_TOTAL_GEAR_RATIO = 1.0 / (double)TOTAL_GEAR_RATIO;

    constexpr double MOTOR_MAX_SPEED_DPS = 3000 * 6; // 1 RPM = 6deg/s
    constexpr double DERATE_RATIO = 0.5;
    constexpr double SLEW_DRIVE_MAX_SPEED_DPS = MOTOR_MAX_SPEED_DPS * INV_TOTAL_GEAR_RATIO * DERATE_RATIO;

    const double slewspeeds[] = {16.0, 32.0, 64.0, 128.0, 256.0, 512.0};
    constexpr unsigned int NUM_SLEW_SPEEDS = sizeof(slewspeeds) / sizeof(double);
    constexpr unsigned int DEFAULT_SLEW_IDX = NUM_SLEW_SPEEDS - 1;

     constexpr double max_slew = SLEW_DRIVE_MAX_SPEED_DPS/ SiderealRate_degpersec;
}

// INDI::IHorizontalCoordinates HorizontalRates_geocentric(double ha, double dec, double lat);