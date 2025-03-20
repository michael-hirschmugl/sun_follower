#include "sun_prediction.hpp"

namespace
{
    /**
     * @brief Number of days before each month in a non-leap year.
     *        Index 0 = January, 1 = February, etc.
     */
    constexpr int DAYS_BEFORE_MONTH[12] = {
        0,   // January
        31,  // February
        59,  // March
        90,  // April
        120, // May
        151, // June
        181, // July
        212, // August
        243, // September
        273, // October
        304, // November
        334  // December
    };

    /**
     * @brief Computes the day of the year for the given date.
     * @param year   The year (e.g., 2025).
     * @param month  The month [1..12].
     * @param day    The day of the month [1..31].
     * @param valid  Optional pointer to a bool that indicates validity.
     * @return Day of the year (1 = January 1), or -1 if invalid.
     */
    inline int dayOfYear(int year, int month, int day, bool* valid = nullptr)
    {
        if (month < 1 || month > 12) {
            if (valid) {
                *valid = false;
            }
            return -1;
        }

        int result = DAYS_BEFORE_MONTH[month - 1] + day;

        // If it's a leap year and the date is after February, add one day
        if (SunPrediction::isLeapYear(year) && (month > 2)) {
            result += 1;
        }

        if (valid) {
            *valid = true;
        }
        return result;
    }

    /**
     * @brief Wraps an angle in degrees to the [0..360) range.
     */
    inline double wrap360(double angleDeg)
    {
        angleDeg = std::fmod(angleDeg, 360.0);
        return (angleDeg < 0.0) ? angleDeg + 360.0 : angleDeg;
    }

    /**
     * @brief Wraps hours to the [0..24) range.
     */
    inline double wrap24(double hourVal)
    {
        hourVal = std::fmod(hourVal, 24.0);
        return (hourVal < 0.0) ? hourVal + 24.0 : hourVal;
    }

} // anonymous namespace

bool SunPrediction::isLeapYear(int year)
{
    // Leap-year rule:
    // - divisible by 400, or
    // - divisible by 4 but not by 100
    return (year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0));
}

int SunPrediction::sunAzimuth(int year, int month, int day,
                              int hour, int minute) const
{
    bool validDate = false;
    int totalDays  = dayOfYear(year, month, day, &validDate);
    if (!validDate) {
        return -1; // Invalid date
    }

    // fhour = hour + minute/60 - 1
    double fhour = static_cast<double>(hour) + (static_cast<double>(minute) / 60.0) - 1.0;

    // Calculate time offset from 31.12.1949 (JD = 32916.5)
    int   delta = (year - 1949);
    int   leap  = (delta / 4);
    double jd   = 32916.5 + (delta * 365 + leap + totalDays) + fhour / 24.0;

    // Correction for secular (century) years that are not leap years
    if ((year % 100 == 0) && (year % 400 != 0)) {
        jd -= 1.0;
    }

    // time = difference from JD(01.01.2000) = 51545.0
    double time   = jd - 51545.0;

    // Mean ecliptic longitude [0..360)
    double mnlong = wrap360(280.460 + 0.9856474 * time);

    // Mean anomaly [0..360) => radians
    double mnanom = wrap360(357.528 + 0.9856003 * time) * RPD;

    // Ecliptic longitude
    double eclong = wrap360(
        mnlong + 1.915 * std::sin(mnanom)
               + 0.020 * std::sin(2.0 * mnanom)
    );
    eclong *= RPD;

    // Earth's axis inclination
    double qblqec = (23.439 - 0.0000004 * time) * RPD;

    // Right ascension (ra)
    double num = std::cos(qblqec) * std::sin(eclong);
    double den = std::cos(eclong);
    double ra  = std::atan(num / den);

    // Quadrant correction for ra
    if (den < 0.0)      ra += PI;
    else if (num < 0.0) ra += TWO_PI;

    // Declination
    double dec = std::asin(std::sin(qblqec) * std::sin(eclong));

    // Greenwich Mean Sidereal Time => wrap to [0..24), then convert to radians
    double gmst = wrap24(6.697375 + 0.0657098242 * time + fhour);
    double lmst = wrap24(gmst + lon_ / 15.0) * (15.0 * RPD);

    // Hour angle
    double ha = lmst - ra;
    if      (ha < -PI) ha += TWO_PI;
    else if (ha >  PI) ha -= TWO_PI;

    // Elevation angle
    double sinEl = std::sin(dec) * std::sin(lat_ * RPD)
                 + std::cos(dec) * std::cos(lat_ * RPD) * std::cos(ha);
    double el = std::asin(sinEl);

    // Azimuth angle
    double sinAz = -std::cos(dec) * std::sin(ha) / std::cos(el);
    double az    = std::asin(sinAz);

    // Quadrant correction
    if ((std::sin(dec) - sinEl * std::sin(lat_ * RPD)) >= 0.0) {
        if (sinAz < 0.0) {
            az += TWO_PI;
        }
    } else {
        az = PI - az;
    }

    // Return azimuth in degrees
    return static_cast<int>(az / RPD);
}
