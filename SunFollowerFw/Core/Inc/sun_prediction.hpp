#pragma once

#include <cmath>

/**
 * @class SunPrediction
 * @brief A lightweight class for calculating the Sun azimuth angle.
 *
 * This class holds fixed coordinates (latitude and longitude) and
 * provides methods to compute the Sun's azimuth angle at a given time.
 */
class SunPrediction
{
public:
    /**
     * @brief Constructs a SunPrediction object with given latitude and longitude.
     * @param latitude  Geographic latitude in degrees.
     * @param longitude Geographic longitude in degrees.
     */
    constexpr SunPrediction(double latitude, double longitude)
        : lat_(latitude), lon_(longitude) {}

    /**
     * @brief Checks if a given year is a leap year.
     * @param year The year to check (e.g., 2024).
     * @return True if it is a leap year, false otherwise.
     */
    static bool isLeapYear(int year);

    /**
     * @brief Computes the Sun azimuth angle for the specified date and time.
     * @param year   The year (e.g., 2025).
     * @param month  The month [1..12].
     * @param day    The day of the month [1..31].
     * @param hour   The hour [0..23].
     * @param minute The minute [0..59].
     * @return The azimuth angle in degrees (as integer),
     *         or -1 if the provided date/time is invalid.
     */
    int sunAzimuth(int year, int month, int day, int hour, int minute) const;

private:
    /// Latitude in degrees
    double lat_;
    /// Longitude in degrees
    double lon_;

    // We define our own PI constant instead of relying on M_PI.
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TWO_PI = PI * 2.0;
    // Radians per degree
    static constexpr double RPD = PI / 180.0;
};
