#include <variant>
#include <Eigen/Core>

#include "multirotor_sim/satellite.h"
#include "multirotor_sim/wsg84.h"

using namespace Eigen;

const double Satellite::GM_EARTH = 3.986005e14;
const double Satellite::OMEGA_EARTH = 7.2921151467e-5;
const double Satellite::PI = 3.1415926535898;
const double Satellite::C_LIGHT = 299792458.0;
const double Satellite::MAXDTOE = 7200.0; // max time difference to GPS Toe (s)
const double Satellite::FREQL1 = 1.57542e9;
const double Satellite::LAMBDA_L1 = Satellite::C_LIGHT / Satellite::FREQL1;



Satellite::Satellite(int id)
{
  carrier_phase_ = 0;
  id_ = id;
  closest_eph_idx_ = 0;
}

void Satellite::update(const GTime &g, const Vector3d &rec_pos, const Vector3d &rec_vel)
{
  double dt = (g - t_last_udpate_).toSec();
  t_last_udpate_ = g;
}

void Satellite::addEphemeris(const eph_t &eph)
{
  ASSERT((eph.sat == id_),
         "Tried to add ephemeris from a different satellite");
  ASSERT((eph_.size() > 0) ? eph.toe >= eph_.back().toe : true,
         "tried to push ephemeris out of order");

  if (eph_.size() > 0 && (eph.toe == eph_.back().toe))
  {
    eph_.back() = eph;
  }
  else
  {
    eph_.push_back(eph);
  }

  if (!se)
  {
    se = &eph_[0];
  }
}

void Satellite::computeMeasurement(const GTime& rec_time, const Vector3d& receiver_pos, const Vector3d& receiver_vel, Vector3d& z)
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
    z(0) = los_to_sat.norm();

    // compute relative velocity between receiver and satellite, adjusted by the clock drift rate
    z(1) = ((sat_vel - receiver_vel).transpose() * los_to_sat / z(0))(0) - C_LIGHT * sat_clk(1);

    // adjust range by the satellite clock offset
    z(0) -= C_LIGHT * sat_clk(0);

    // Compute Azimuth and Elevation to satellite
    Vector2d az_el;
    los2azimuthElevation(receiver_pos, los_to_sat, az_el);
    Vector3d lla = WSG84::ecef2lla(receiver_pos);

    // Compute and incorporate ionospheric delay
    double ion_delay = ionosphericDelay(rec_time, lla, az_el);
    z(0) += ion_delay;

    z(2) = los_to_sat.norm() / LAMBDA_L1;

    return;
}

void Satellite::los2azimuthElevation(const Vector3d& receiver_pos_ecef, const Vector3d& los_ecef, Vector2d& az_el)
{
    xform::Xformd x_e2n = WSG84::x_ecef2ned(receiver_pos_ecef);
    Vector3d los_ned = x_e2n.q().rotp(los_ecef.normalized());
    quat::Quatd q_los = quat::Quatd::from_two_unit_vectors(e_x, los_ned);
    az_el(0) = q_los.yaw();
    az_el(1) = q_los.pitch();
}

double Satellite::ionosphericDelay(const GTime& gtime, const Vector3d& lla, const Vector2d& az_el)
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
    double t= 43200*lambda_I + gtime.tow_sec;
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

double Satellite::selectEphemeris(const GTime &time)
{
  // find the closest ephemeris
  double dt = INFINITY;
  assert(eph_.size() > 0);
  while (1)
  {
    dt = (time - se->toe).toSec();
    if (dt < 0 && (closest_eph_idx_ > 0))
    {
      double prev_dt = (time - eph_[closest_eph_idx_-1].toe).toSec();
      if (std::abs(prev_dt) < std::abs(dt))
      {
        dt = prev_dt;
        closest_eph_idx_ -= 1;
        se = &eph_[closest_eph_idx_];
      }
      else
        break;
    }

    else if (dt > 0 && (closest_eph_idx_ < eph_.size()-1))
    {
      double next_dt = (time - eph_[closest_eph_idx_+1].toe).toSec();
      if (std::abs(next_dt) < std::abs(dt))
      {
        dt = next_dt;
        closest_eph_idx_ += 1;
        se = &eph_[closest_eph_idx_];
      }
      else
        break;
    }
    else
      break;
  }
  return dt;
}


bool Satellite::computePositionVelocityClock(const GTime& time, const Ref<Vector3d> &_pos, const Ref<Vector3d> &_vel, const Ref<Vector2d>& _clock)
{
    // const-cast hackery to get around Ref
    Ref<Vector3d> pos = const_cast<Ref<Vector3d>&>(_pos);
    Ref<Vector3d> vel = const_cast<Ref<Vector3d>&>(_vel);
    Ref<Vector2d> clock = const_cast<Ref<Vector2d>&>(_clock);

    double dt = selectEphemeris(time);

    if (dt > MAXDTOE)
      return false;

    // https://www.ngs.noaa.gov/gps-toolbox/bc_velo/bc_velo.c
    double n0 = std::sqrt(GM_EARTH/(se->A*se->A*se->A));
    double tk = dt;
    double n = n0 + se->deln;
    double mk = se->M0 + n*tk;
    double mkdot = n;
    double ek = mk;
    double ek_prev = 0.0;

    int i = 0;
    while (ek - ek_prev > 1e-8 && i < 7)
    {
        ek_prev = ek;
        ek = mk + se->e*std::sin(ek);
        i++;
    }
    double sek = std::sin(ek);
    double cek = std::cos(ek);


    double ekdot = mkdot/(1.0 - se->e * cek);

    double tak = std::atan2(std::sqrt(1.0-se->e*se->e) * sek, cek - se->e);
    double takdot = sek*ekdot*(1.0+se->e*std::cos(tak)) / (std::sin(tak)*(1.0-se->e*cek));


    double phik = tak + se->omg;
    double sphik2 = std::sin(2.0 * phik);
    double cphik2 = std::cos(2.0 * phik);
    double corr_u = se->cus * sphik2 + se->cuc * cphik2;
    double corr_r = se->crs * sphik2 + se->crc * cphik2;
    double corr_i = se->cis * sphik2 + se->cic * cphik2;
    double uk = phik + corr_u;
    double rk = se->A*(1.0 - se->e*cek) + corr_r;
    double ik = se->i0 + se->idot*tk + corr_i;

    double s2uk = std::sin(2.0*uk);
    double c2uk = std::cos(2.0*uk);

    double ukdot = takdot + 2.0 * (se->cus * c2uk - se->cuc*s2uk) * takdot;
    double rkdot = se->A * se->e * sek * n / (1.0 - se->e * cek) + 2.0 * (se->crs * c2uk - se->crc * s2uk) * takdot;
    double ikdot = se->idot + (se->cis * c2uk - se->cic * s2uk) * 2.0 * takdot;

    double cuk = std::cos(uk);
    double suk = std::sin(uk);

    double xpk = rk * cuk;
    double ypk = rk * suk;

    double xpkdot = rkdot * cuk - ypk * ukdot;
    double ypkdot = rkdot * suk + xpk * ukdot;

    double omegak = se->OMG0 + (se->OMGd - OMEGA_EARTH) * tk - OMEGA_EARTH * se->toes;
    double omegakdot = se->OMGd - OMEGA_EARTH;

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

    tk = (time - se->toc).toSec();
    double dts = se->f0 + se->f1*tk + se->f2*tk*tk;

    // Correct for relativistic effects on the satellite clock
    dts -= 2.0*std::sqrt(GM_EARTH * se->A) * se->e * sek/(C_LIGHT * C_LIGHT);

    clock(0) = dts; // satellite clock bias
    clock(1) = se->f1 + se->f2*tk; // satellite drift rate

    return true;
}

void Satellite::readFromRawFile(std::string filename)
{
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  const int size = sizeof(eph_t);
  char buf[size];
  eph_t* eph = (eph_t*)buf;

  if (!file.is_open())
  {
    std::cout << "unable to open " << filename << std::endl;
    return;
  }

  while (!file.eof())
  {
    file.read(buf, size);
    if (!file)
      break;
    eph->toe.week = eph->toe.week - DateTime::GPS_UTC_OFFSET_SEC;
    eph->toe.tow_sec = eph->toe.week % DateTime::SECONDS_IN_WEEK - DateTime::LEAP_SECONDS;
    eph->toe.week /= DateTime::SECONDS_IN_WEEK;
    if (eph->sat == id_)
      addEphemeris(*eph);
  }

}
