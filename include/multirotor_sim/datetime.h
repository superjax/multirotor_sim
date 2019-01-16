#pragma once

#include "multirotor_sim/gtime.h"

class GTime;
class DateTime
{
public:
    static constexpr int SECONDS_IN_WEEK = 604800.0;
    static constexpr int SECONDS_IN_HALF_WEEK = 302400.0;
    static constexpr int SECONDS_IN_DAY = 86400.0;
    static constexpr int SECONDS_IN_HOUR = 3600.0;
    static constexpr int SECONDS_IN_MINUTE = 60.0;
    static constexpr int LEAP_SECONDS = 18;

    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;

    DateTime();
    DateTime(const GTime& g);
    DateTime& operator= (const GTime& g);
    GTime toGTime() const;
};
