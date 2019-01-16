#include <stdio.h>
#include <Eigen/Core>

#include "multirotor_sim/ephemeris.h"
#include "multirotor_sim/wsg84.h"

using namespace Eigen;

Ephemeris::Ephemeris() {}

void Ephemeris::computePseudorange(const GTime& rec_time, const Vector3d& receiver_pos, const Vector3d& receiver_vel, double& pseudorange, double& range_rate) const
{
    Vector3d sat_pos, sat_vel;
    Vector2d sat_clk;
    computePositionVelocityClock(rec_time, sat_pos, sat_vel, sat_clk);
    Vector3d los_to_sat = sat_pos - receiver_pos;
    double tau = los_to_sat.norm() / C_LIGHT;  // Time it took for observation to get here

    // extrapolate satellite position backwards in time
    sat_pos -= sat_vel * tau;

    // Earth rotation correction. The change in velocity can be neglected.
    double xrot = sat_pos.x() + sat_pos.y() * OMEGA_EARTH * tau;
    double yrot = sat_pos.y() - sat_pos.x() * OMEGA_EARTH * tau;
    sat_pos.x() = xrot;
    sat_pos.y() = yrot;

    // Re-calculate the line-of-sight vector with the adjusted position
    los_to_sat = sat_pos - receiver_pos;
    double range = los_to_sat.norm();

    // compute relative velocity between receiver and satellite, adjusted by the clock drift rate
    range_rate = ((sat_vel - receiver_vel).transpose() * los_to_sat / range)(0) - C_LIGHT * sat_clk(1);

    // adjust range by the satellite clock offset
    range -= C_LIGHT * sat_clk(0);

    // Compute Azimuth and Elevation to satellite
    Vector2d az_el;
    los2azimuthElevation(receiver_pos, los_to_sat, az_el);

    // Compute and incorporate ionospheric delay
    range += ionosphericDelay(rec_time, WSG84::ecef2lla(receiver_pos), az_el);

    return;
}

void Ephemeris::los2azimuthElevation(const Vector3d& receiver_pos_ecef, const Vector3d& los_ecef, Vector2d& az_el) const
{
    xform::Xformd x_e2n = WSG84::x_ecef2ned(receiver_pos_ecef);
    Vector3d los_ned = x_e2n.q().rotp(los_ecef.normalized());
    quat::Quatd q_los = quat::Quatd::from_two_unit_vectors(e_x, los_ned);
    az_el(0) = q_los.yaw();
    az_el(1) = q_los.pitch();
}

double Ephemeris::ionosphericDelay(const GTime& gtime, const Vector3d& lla, const Vector2d& az_el) const
{
    // Klobuchar Algorithm:
    // https://gssc.esa.int/navipedia/index.php/Klobuchar_Ionospheric_Model

    const double ion[]={ /* 2004/1/1 */
            0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
            0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
        };

    // Earth-Centered Angle (Elevation in Semicircles)
    double psi = 0.0137 / (az_el(1)/M_PI + 0.11) - 0.022;

    // latitude of the ionosphere pierce point (IPP)
    double phi_I = lla(0)/M_PI + psi * std::cos(az_el(0));
    phi_I = phi_I > 0.416 ? 0.416 : phi_I < -0.416 ? -0.416 : phi_I;

    // longitude of IPP
    double lambda_I = lla(1)/M_PI + (psi * std::sin(az_el(0))/cos(phi_I*M_PI));

    // geomagnetic latitude of the IPP
    double phi_m = phi_I + 0.064 * std::cos((lambda_I - 1.617) * M_PI);

    // local time at hte IPP
    double t= 43200*lambda_I + gtime.sec;
    t -= std::floor(t/86400.0) * 86400.0;

    // Amplitude of Ionospheric Delay
    double Amp = ion[0] + phi_m * (ion[1] + phi_m * (ion[2] + phi_m * ion[3]));
    Amp = Amp < 0 ? 0 : Amp;

    // Period of Ionospheric Delay
    double Per = ion[4] + phi_m * (ion[5] + phi_m * (ion[6] + phi_m * ion[7]));
    Per = Per < 72000 ? 72000 : Per;

    // Phase of Ionospheric Delay
    double X_I = 2.0 * M_PI * (t - 50400.0) / Per;

    // Slant Factor
    double F = 1.0 + 16.0 * pow((0.53 - az_el(1)/M_PI), 3.0);


    // Compute Ionospheric Time Delay (meters)
    if (std::abs(X_I) <= 1.57)
    {
        double X2 = X_I*X_I;
        double X4 = X2*X2;
        return C_LIGHT * F * (5e-9 + Amp * (1.0 - X2/2.0 + X4/24.0));
    }
    else
        return C_LIGHT * F * 5e-9;
}


void Ephemeris::computePositionVelocityClock(const GTime& time, Vector3d &pos, Vector3d &vel, Vector2d& clock) const
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

    // Correct for relativistic effects on the satellite clock
    dts -= 2.0*std::sqrt(GM_EARTH * A) * e * sek/(C_LIGHT * C_LIGHT);

    clock(0) = time.sec - dts; // corrected satellite ToW
    clock(1) = f1 + f2*tk; // satellite drift rate
}
