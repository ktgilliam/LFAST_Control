#include "astro_math.h"
#include <cmath>

#include "libastro.h"

INDI::IHorizontalCoordinates HorizontalRates_geocentric(double ha, double dec, double lat)
{
    double ha_rad = hrs2rad(ha);
    double cHA = rad2deg(std::cos(ha_rad));
    double sHA = rad2deg(std::sin(ha_rad));
    double cDEC = cosd(dec);
    double sDEC = sind(dec);
    double cLAT = cosd(lat);
    double sLAT = sind(lat);

    //
    // Angular Positions
    //

    // Altitude Angle
    double altTerm1 = sDEC * sLAT;
    double altTerm2 = cDEC * cLAT * cHA;
    double sALT = altTerm1 + altTerm2;
    double cALT = std::sqrt(1-sALT*sALT);

    // Azimuth Angle
    double azY = -1.0 * sHA * cDEC;
    double azX = sDEC * cLAT - cDEC * sLAT * cHA;
    double azDen = (azY*azY)+(azX*azX);
    double cAZ = azX/azDen;
    double sAZ = azY/azDen;
    // double azAngle_deg = atan2d(azNum, azDen);
    // double cAZ = cosd(azAngle_deg);

    // Parallactic Angle
    double parY = sHA * cLAT;
    double parX = sLAT * cDEC - sDEC * cLAT * cHA;
    double parDen = (parY*parY)+(parX*parX);
    double cPAR = parX/parDen;
    double sPAR = parY/parDen;

    //
    // Angular Rates
    // Note: For now disregarding non-sidereal targets (proper motion terms are ignored).

    // Altitude Rate
    double altArg1 = cLAT * sAZ;
    // double altArg2 = (proper motion derivative of non-sidereal target);
    double altRate_dps = LFAST::SiderealRate_degpersec * altArg1;

    // Azimuth Rate
    double azArg1 = cDEC * cPAR / cALT;
    // double azArg2 = (proper motion derivative of non-sidereal target);
    double azRate_dps = LFAST::SiderealRate_degpersec * azArg1;

    // // Parallactic Rate
    // double parArg1 = -1.0 * cLAT * cAZ / cALT;
    // // double parArg2 = (proper motion derivative of non-sidereal target);
    // azRate_radpersec = LFAST::SiderealRate_degpersec * parArg1;
    INDI::IHorizontalCoordinates vHz {0,0};
    vHz.altitude = altRate_dps;
    vHz.azimuth = azRate_dps;
    return vHz;
}