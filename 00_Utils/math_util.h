#pragma once
#include <cmath>

#ifdef M_PI
#undef M_PI
#endif
#define M_PI 3.1415926535897932384626433832795

#ifdef M_2_PI
#undef M_2_PI
#endif
#define M_2_PI 6.283185307179586476925286766559 /* pi*2 */

#ifdef M_PI_2
#undef M_PI_2
#endif
#define M_PI_2		1.57079632679489661923	/* pi/2 */

#ifdef M_SQRT2
#undef M_SQRT2
#endif
#define M_SQRT2 1.4142135623730950488016887

#ifndef EULER
#define EULER 2.71828182845904523536028747135266249775724709369995
#endif
#ifndef ROOT2
#define ROOT2 1.41421356237309504880168872420969807856967187537694
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105



#define INV_180    0.00555555555555555555555555555556 // 1/180
#define INV_3600   0.00027777777777777777777777777778
#define INV_PI     0.31830988618379067153776752674502
#define INV_2_PI   0.15915494309189533576888376337251
#define INV_24     0.04166666666666666666666666666666

template <typename T, typename U>
inline T ulim(T val, U upper)
{
    return val < upper ? val : upper;
}

template <typename T, typename U>
inline T llim(T val, U lower)
{
    return val > lower ? val : lower;
}

template <typename T, typename U, typename V>
inline T saturate(T val, U lower, V upper)
{
    T val1 = llim(val, lower);
    T val2 = ulim(val1, upper);

    // T val1 = val > lower ? val : lower;
    // T val2 = val1 < upper ? val1 : upper;
    return val2;
}

template <typename T>
inline int sign(T val)
{
    if (val == 0.0)
        return 0.0;
    else
        return std::signbit(val) ? -1 : 1;
}

template <typename T>
inline double hrs2rad(T val)
{
    return val * M_2_PI * INV_24;
}

template <typename T>
inline double rad2hrs(T val)
{
    return val * 24.0 * INV_2_PI;
}

template <typename T>
inline double rad2deg(T val)
{
    return val * 180.0 * INV_PI;
}

template <typename T>
inline double deg2rad(T val)
{
    return val * M_PI  * INV_180;
}


template <typename T>
inline double arcsec2deg(T val)
{
    return val * INV_3600;
}

template <typename T>
inline double deg2arcsec(T val)
{
    return val * 3600.0;
}

template <typename T>
inline double arcsec2rad(T val)
{
    return val * INV_3600 * INV_180 * M_PI;
}

template <typename T>
inline double rad2arcsec(T val)
{
    return val * 3600.0 * 180.0 * INV_PI;
}

template <typename T>
inline double radpersec2RPM(T val)
{
    return val * 30.0 * INV_PI;
}

template <typename T>
inline double RPM2radpersec(T val)
{
    return val * 30.0 * INV_PI;
}