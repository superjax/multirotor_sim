#pragma once
#include <stdint.h>
#include <vector>

#include <Eigen/Core>

#include "multirotor_sim/datetime.h"
#include "multirotor_sim/gtime.h"

typedef struct {
  int32_t sat; // satellite number
  int32_t iode; // IODE Issue of Data, Ephemeris (ephemeris version)
  int32_t iodc; // IODC Issue of Data, Clock (clock version)
  int32_t sva; // SV accuracy (URA index) IRN-IS-200H p.97
  int32_t svh; // SV health GPS/QZS (0:ok)
  int32_t week; // GPS/QZS: gps week, GAL: galileo week (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) // GPS/QZS: code on L2 * GAL/CMP: data sources
  int32_t code;  // GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type
  int32_t flag;
  GTime toe; // Toe
  GTime toc; // clock data reference time (s) (20.3.4.5)
  GTime ttr; // T_trans
  double A; // Semi-Major Axis m
  double e; // Eccentricity (no units)
  double i0; // Inclination Angle at Reference Time (rad)
  double OMG0; // Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)
  double omg; // Argument of Perigee (rad)
  double M0; // Mean Anomaly at Reference Time (rad)
  double deln; // Mean Motion Difference From Computed Value (rad)
  double OMGd; // Rate of Right Ascension (rad/s)
  double idot; // Rate of Inclination Angle (rad/s)
  double crc; // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
  double crs; // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)
  double cuc; // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad)
  double cus; // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)
  double cic; // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)
  double cis; // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)
  double toes; // Reference Time Ephemeris in week (s)
  double fit; // fit interval (h) (0: 4 hours, 1:greater than 4 hours)
  double f0; // SV clock parameters - af0
  double f1; // SV clock parameters - af1
  double f2; // SV clock parameters - af2 * * GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) // group delay parameter * GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E * CMP    :tgd[0]=BGD1,tgd[1]=BGD
  double tgd[4];
  double Adot; // Adot for CNAV
  double ndot; // ndot for CNAV
} eph_t;

using namespace Eigen;

class Satellite
{
public:
    static constexpr double GM_EARTH = 3.986005e14;
    static constexpr double OMEGA_EARTH = 7.2921151467e-5;
    static constexpr double PI = 3.1415926535898;
    static constexpr double C_LIGHT = 299792458.0;
    static constexpr double MAXDTOE = 7200.0; // max time difference to GPS Toe (s)

    Satellite();
    void update(const GTime &g, const Vector3d& rec_pos, const Vector3d& rec_vel);
    bool computePositionVelocityClock(const GTime &g, const Ref<Vector3d> &pos, const Ref<Vector3d> &vel, const Ref<Vector2d> &clock);
    void computeMeasurement(const GTime& rec_time, const Vector3d& receiver_pos, const Vector3d &receiver_vel, Vector3d &z);
    void los2azimuthElevation(const Vector3d& receiver_pos_ecef, const Vector3d& los_ecef, Vector2d& az_el);
    double ionosphericDelay(const GTime &t, const Vector3d& lla, const Vector2d& az_el);
    double selectEphemeris(const GTime& time);
    void readFromRawFile(std::string filename);
    void addEphemeris(const eph_t& eph);


    GTime t_last_udpate_;
    Vector2d az_el_;
    Vector3d rec_pos_;
    Vector3d rec_vel_;
    Vector3d sat_pos_;
    Vector3d sat_vel_;

    std::vector<eph_t> eph_;
    const eph_t* se = nullptr;
    int closest_eph_idx_;

    double carrier_phase_;
    double carrier_phase_rate_;
    double pseudorange_;
    double pseudorange_rate_;
};
