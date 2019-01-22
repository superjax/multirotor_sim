#include <gtest/gtest.h>

#include "multirotor_sim/satellite.h"
#include "multirotor_sim/test_common.h"
#include "multirotor_sim/wsg84.h"

class TestSatellite : public ::testing::Test
{
protected:
  void SetUp() override
    {
      time.week = 86400.00 / DateTime::SECONDS_IN_WEEK;
      time.tow_sec = 86400.00 - (time.week * DateTime::SECONDS_IN_WEEK);

      eph.A = 5153.79589081 * 5153.79589081;
      eph.toe.week = 93600.0 / DateTime::SECONDS_IN_WEEK;
      eph.toe.tow_sec = 93600.0 - (eph.toe.week * DateTime::SECONDS_IN_WEEK);
      eph.toes = 93600.0;
      eph.deln =  0.465376527657e-08;
      eph.M0 =  1.05827953357;
      eph.e =  0.00223578442819;
      eph.omg =  2.06374037770;
      eph.cus =  0.177137553692e-05;
      eph.cuc =  0.457651913166e-05;
      eph.crs =  88.6875000000;
      eph.crc =  344.96875;
      eph.cis = -0.856816768646e-07;
      eph.cic =  0.651925802231e-07;
      eph.idot =  0.342514267094e-09;
      eph.i0 =  0.961685061380;
      eph.OMG0 =  1.64046615454;
      eph.OMGd = -0.856928551657e-08;
  }
  GTime time;
  Satellite eph;
};

TEST_F (TestSatellite, CheckSatPositionVelocityClock)
{
    Vector3d oracle_pos, oracle_vel, oracle_pos2;
    Vector3d new_pos, new_vel;
    Vector3d truth_pos, truth_vel;
    double oracle_clock, oracle_clock2, oracle_clk_rate;
    double dt = 1e-3;
    GTime t2 = time + dt;
    truth_pos << -12611434.19782218519,
            -13413103.97797041226,
            19062913.07357876760;
    truth_vel <<  266.280379332602,
            -2424.768347293139,
            -1529.762077704072;
    eph2pos(time, &eph, oracle_pos, &oracle_clock);
    eph2pos(t2, &eph, oracle_pos2, &oracle_clock2);
    oracle_vel = (oracle_pos2 - oracle_pos) / dt;
    oracle_clk_rate = (oracle_clock2 - oracle_clock) / dt;

    Vector2d clock;
    eph.computePositionVelocityClock(time, new_pos, new_vel, clock);

    EXPECT_MAT_NEAR(oracle_pos, new_pos, 1e-5);
    EXPECT_MAT_NEAR(oracle_vel, new_vel, 1e-3);
    EXPECT_MAT_NEAR(new_pos, truth_pos, 1e-5);
    EXPECT_MAT_NEAR(new_vel, truth_vel, 1e-5);

    EXPECT_NEAR(clock(0), oracle_clock, 1e-8);
    EXPECT_NEAR(clock(1), oracle_clk_rate, 1e-8);
}

TEST_F (TestSatellite, AzimuthElevationStraightUp)
{
    Vector2d az_el, clock;
    Vector3d sat_pos, sat_vel;
    eph.computePositionVelocityClock(time, sat_pos, sat_vel, clock);
    Vector3d sat_lla = WSG84::ecef2lla(sat_pos);
    Vector3d surface_lla = sat_lla;
    surface_lla(2) = 0;
    Vector3d surface_ecef = WSG84::lla2ecef(surface_lla);

    Vector3d los_ecef = sat_pos - surface_ecef;

    eph.los2azimuthElevation(surface_ecef, los_ecef, az_el);

    ASSERT_NEAR(az_el(1), M_PI/2.0, 1e-7);
}

TEST_F (TestSatellite, AzimuthElevationProvo)
{
    Vector2d az_el, clock;
    Vector3d sat_pos, sat_vel;
    eph.computePositionVelocityClock(time, sat_pos, sat_vel, clock);
    Vector3d sat_lla = WSG84::ecef2lla(sat_pos);

    Vector3d provo_lla{40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511};
    Vector3d provo_ecef = WSG84::lla2ecef(provo_lla);

    Vector3d los_ecef = sat_pos - provo_ecef;

    eph.los2azimuthElevation(provo_ecef, los_ecef, az_el);

    ASSERT_NEAR(az_el(0), -1.09260980, 1e-8);
    ASSERT_NEAR(az_el(1), 1.18916781, 1e-8);
}

TEST_F (TestSatellite, IonoshereCalculation)
{
    Vector2d az_el, clock;
    Vector3d sat_pos, sat_vel;
    eph.computePositionVelocityClock(time, sat_pos, sat_vel, clock);
    Vector3d provo_lla{40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511};
    Vector3d provo_ecef = WSG84::lla2ecef(provo_lla);
    Vector3d los_ecef = sat_pos - provo_ecef;
    eph.los2azimuthElevation(provo_ecef, los_ecef, az_el);

    double oracle_ion_delay = ionmodel(time, provo_lla.data(), az_el.data());
    double new_ion_delay = eph.ionosphericDelay(time, provo_lla, az_el);

    ionoutc_t ion = {true, true,
                     0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
                     0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07};
    double sdr_lib_delay = ionosphericDelay(&ion, time, provo_lla.data(), az_el.data());

    ASSERT_NEAR(oracle_ion_delay, new_ion_delay, 1e-8);
    ASSERT_NEAR(sdr_lib_delay, new_ion_delay, 1e-8);
}

TEST_F (TestSatellite, PsuedorangeSim)
{
    Vector3d provo_lla{40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511};
    Vector3d provo_ecef = WSG84::lla2ecef(provo_lla);
    Vector3d rec_vel = Vector3d::Constant(0);

    Vector3d z;

    eph.computeMeasurement(time, provo_ecef, rec_vel, z);

    ionoutc_t ion = {true, true,
                     0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
                     0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07};
    range_t rho;
    computeRange(&rho, eph, &ion, time, provo_ecef);

    EXPECT_NEAR(rho.range, z(0), 1e-5);
    EXPECT_NEAR(rho.rate, z(1), 1e-5);
}
