#include <cmath>
#include "multirotor_sim/gtime.h"
#include "multirotor_sim/datetime.h"

GTime::GTime()
{}

GTime::GTime(int week, double sec) :
    week{week},
    sec{sec}
{}

GTime::GTime(const DateTime &t)
{
    (*this) = t.toGTime();
}

GTime& GTime::operator= (const DateTime& t)
{
    (*this) = t.toGTime();
    return (*this);
}

GTime GTime::operator -(const GTime& gt2) const
{
    int dweek = week - gt2.week;
    double dsec = sec - gt2.sec;

    return GTime{dweek, dsec};
}

GTime GTime::operator +(const GTime& gt2) const
{
    int nweek = week + gt2.week;
    double nsec = sec + gt2.sec;

    if (nsec >= DateTime::SECONDS_IN_WEEK)
    {
        nsec -= DateTime::SECONDS_IN_WEEK;
        nweek += 1;
    }
    return GTime{nweek, nsec};
}

DateTime GTime::toDate() const
{
    double s_leap = sec - DateTime::LEAP_SECONDS;
    int c = (int)(7*week + std::floor(s_leap/86400.0)+2444245.0) + 1537;
    int d = (int)((c-122.1)/365.25);
    int e = 365*d + d/4;
    int f = (int)((c-e)/30.6001);

    DateTime t;
    t.day = c - e - (int)(30.6001*f);
    t.month = f - 1 - 12*(f/14);
    t.year = d - 4715 - ((7 + t.month)/10);

    t.hour = ((int)(s_leap/3600.0))%24;
    t.minute = ((int)(s_leap/60.0))%60;
    t.second = s_leap - 60.0*floor(s_leap/60.0);
    return t;
}

double GTime::toSec() const
{
    return sec + DateTime::SECONDS_IN_WEEK * week;
}

GTime GTime::operator+(const double& dsec)
{
    GTime nt = *this;
    nt += dsec;
    return nt;
}

GTime& GTime::operator +=(const double& dsec)
{
    sec += dsec;
    if (sec > DateTime::SECONDS_IN_WEEK)
    {
        sec -= DateTime::SECONDS_IN_WEEK;
        week += 1;
    }
    return *this;
}
