#pragma once
#include <stdint.h>

#include "multirotor_sim/datetime.h"

class DateTime;
class GTime
{
public:
    int64_t week;
    double tow_sec;

    GTime();
    GTime(int64_t week, double tow_sec);
    GTime(const DateTime& t);
    GTime& operator= (const DateTime& t);
    GTime operator -(const GTime& gt2) const;
    GTime operator +(const GTime& gt2) const;
    GTime operator +(const double& tow_sec);
    GTime& operator +=(const double& tow_sec);
    DateTime toDate() const;
    double toSec() const;
};
