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
}

// INDI::IHorizontalCoordinates HorizontalRates_geocentric(double ha, double dec, double lat);