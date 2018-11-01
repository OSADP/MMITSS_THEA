/* geoCoord is a utility class to translate between WGS84 and ECEF
 * Reference:
 *
 * Modified by Yiheng Feng 2013.9.23
 * Function: ecef2local added!
 */
#pragma once

#include <cmath>

class geoCoord
{
  private:
    // parameters for conversion
    double a; // meters
    double b; // meters
    double f;
    double inv_f;
    double e;
    double pi;

    // longitude and latitude origin
    double longitude_init{0};
    double latitude_init{0};
    double altitude_init{0};
    double R_sinLat{0};
    double R_cosLat{0};
    double R_sinLong{0};
    double R_cosLong{0};

    // longutude and latitude in radians
    double longitude_r{0}, latitude_r{0};

  public:
    // longitude, latitude, and altitude
    // double longitude, latitude, altitude ; /* decimal representation */
    double longitude_degrees{0};
    double longitude_minutes{0};
    double longitude_seconds{0};
    int latitude_degrees{0};
    int latitude_minutes{0};
    double latitude_seconds{0};

    double ex_init{0};
    double ey_init{0};
    double ez_init{0};

    geoCoord()
    {
        a = 6378137.0;     // meters
        b = 6356752.3142;  // meters
        f = 1.0 - (b / a); //(a-b)/a ;
        inv_f = 1.0 / f;
        e = sqrt(f * (2 - f));
        pi = 4.0 * atan(1.0);
    }

  public:
    /* init(...) used to set the base location used for all transformations and represents the
     * origin in the local tangent plane.
     * This method must be called prior to any other conversion.
     */
    void init(double longitude, double latitude, double altitude);

    void local2ecef(double x, double y, double z, double* ex, double* ey, double* ez);

    void ecef2local(double ex, double ey, double ez, double* x, double* y, double* z);

    /* dms2d(...) converts a position from degree, minutes, seconds to degrees */
    double dms2d(double degree, double minutes, double seconds);

    /* lla2ecef(...) converts longitude, latitude, and altitude to earth-centered, earth-fixed
     * using the base location as the origin
     */
    void lla2ecef(
        double longitude, double latitude, double altitude, double* x, double* y, double* z);

    /* ecef2lla(...) converts from earth-center, earth-fixed to longitude, latitude, and altitude
     * in degrees
     */
    void ecef2lla(
        double x, double y, double z, double* longitude, double* latitude, double* altitude);

    // double d2dms(double degree, int idegree, double minutes, double seconds);
};
