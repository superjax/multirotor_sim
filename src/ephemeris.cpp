#include <stdio.h>
#include <Eigen/Core>

#include "multirotor_sim/ephemeris.h"

using namespace Eigen;

Ephemeris::Ephemeris() {}

void Ephemeris::computePositionVelocityClock(const GTime& time, Vector3d &pos, Vector3d &vel, Vector2d& clock)
{
    // https://www.ngs.noaa.gov/gps-toolbox/bc_velo/bc_velo.c
    double n0 = std::sqrt(GM_EARTH/(A*A*A));
    double tk = (time - toe).toSec();
    double n = n0 + deln;
    double mk = M0 + n*tk;
    double mkdot = n;
    double ek = mk;
    double ek_prev = 0.0;

    int i = 0;
    while (ek - ek_prev > 1e-8 && i < 7)
    {
        ek_prev = ek;
        ek = mk + e*std::sin(ek);
        i++;
    }
    double sek = std::sin(ek);
    double cek = std::cos(ek);

    double ekdot = mkdot/(1.0 - e * cek);

    double tak = std::atan2(std::sqrt(1.0-e*e) * sek, cek - e);
    double takdot = sek*ekdot*(1.0+e*std::cos(tak)) / (std::sin(tak)*(1.0-e*cek));

    double phik = tak + omg;
    double sphik2 = std::sin(2.0 * phik);
    double cphik2 = std::cos(2.0 * phik);
    double corr_u = cus * sphik2 + cuc * cphik2;
    double corr_r = crs * sphik2 + crc * cphik2;
    double corr_i = cis * sphik2 + cic * cphik2;
    double uk = phik + corr_u;
    double rk = A*(1.0 - e*cek) + corr_r;
    double ik = i0 + idot*tk + corr_i;

    double s2uk = std::sin(2.0*uk);
    double c2uk = std::cos(2.0*uk);

    double ukdot = takdot + 2.0 * (cus * c2uk - cuc*s2uk) * takdot;
    double rkdot = A * e * sek * n / (1.0 - e * cek) + 2.0 * (crs * c2uk - crc * s2uk) * takdot;
    double ikdot = idot + (cis * c2uk - cic * s2uk) * 2.0 * takdot;

    double cuk = std::cos(uk);
    double suk = std::sin(uk);

    double xpk = rk * cuk;
    double ypk = rk * suk;

    double xpkdot = rkdot * cuk - ypk * ukdot;
    double ypkdot = rkdot * suk + xpk * ukdot;

    double omegak = OMG0 + (OMGd - OMEGA_EARTH) * tk - OMEGA_EARTH * toes;
    double omegakdot = OMGd - OMEGA_EARTH;

    double cwk = std::cos(omegak);
    double swk = std::sin(omegak);
    double cik = std::cos(ik);
    double sik = std::sin(ik);

    pos.x() = xpk * cwk - ypk * swk * cik;
    pos.y() = xpk * swk + ypk * cwk * cik;
    pos.z() = ypk * sik;

    vel.x() = ( xpkdot - ypk*cik*omegakdot )*cwk
            - ( xpk*omegakdot + ypkdot*cik - ypk*sik*ikdot )*swk;
    vel.y() = ( xpkdot - ypk*cik*omegakdot )*swk
            + ( xpk*omegakdot + ypkdot*cik - ypk*sik*ikdot )*cwk;
    vel.z() = ypkdot*sik + ypk*cik*ikdot;

    tk = (time - toc).toSec();
    double dts = f0 + f1*tk + f2*tk*tk;
    dts -= 2.0*std::sqrt(GM_EARTH * A) * e * sek/(C_LIGHT * C_LIGHT);

    clock(0) = time.sec - dts; // corrected satellite ToW
    clock(1) = f1 + f2*tk; // satellite drift rate
}
