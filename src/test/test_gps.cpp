#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/utils.h"
#include "multirotor_sim/test_common.h"

static const double RAD2DEG = 180.0 / M_PI;
static const double DEG2RAD = M_PI / 180.0;


TEST (GPS, lla2ecef)
{
    Vector3d lla = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511}; // BYU Campus
    Vector3d ecef_known = {-1798810.23, -4532232.54, 4099784.74};

    Vector3d ecef_calc = WSG84::lla2ecef(lla);

    ASSERT_MAT_NEAR(ecef_known, ecef_calc, 1e-2);
}

TEST (GPS, ecef2lla)
{
    Vector3d ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vector3d lla_known = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.998309};

    Vector3d lla_calc = WSG84::ecef2lla(ecef);

    ASSERT_MAT_NEAR(lla_known, lla_calc, 1e-6);
}

TEST (GPS, ecef2lla2ecef)
{
    Vector3d ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vector3d lla_known = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.998309};

    Vector3d lla_calc = WSG84::ecef2lla(ecef);
    Vector3d ecef_calc = WSG84::lla2ecef(lla_calc);

    ASSERT_MAT_NEAR(ecef_calc, ecef, 1e-6);
}

TEST (GPS, ecef2ned)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);
    Vector3d ned1 = {-54.976484, 1.276565, 0.000237};

    xform::Xformd x_e2n = WSG84::ecef2ned(ecef0);
    Vector3d ned1_hat = x_e2n.transformp(ecef1);

    EXPECT_MAT_NEAR(ned1_hat, ned1, 1e-3);
}

TEST (GPS, ecef2ned_check_axes)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);
    xform::Xformd x_e2n = WSG84::ecef2ned(ecef0);

    double sp = std::sin(lla0(0));
    double cp = std::cos(lla0(0));
    double sl = std::sin(lla0(1));
    double cl = std::cos(lla0(1));

    Matrix3d R; // https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
    R << -sp*cl, -sl, -cp*cl,
         -sp*sl,  cl, -cp*sl,
          cp,     0,  -sp;
    EXPECT_MAT_NEAR(x_e2n.q().R(), R.transpose(), 1e-8);

    Vector3d E_r_N_E = 1.0*ecef0;
    E_r_N_E /= E_r_N_E.stableNorm();
    Vector3d E_z_N = x_e2n.q().rota(e_z);
    EXPECT_MAT_NEAR(-1.0 * E_z_N, E_r_N_E, 3e-3);
}

TEST (GPS, ned2ecef)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d ned1 = {-54.976484, 1.276565, 0.000237};
    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);

    xform::Xformd x_e2n = WSG84::ecef2ned(ecef0);
    Vector3d ned_hat = x_e2n.transformp(ecef1);

    EXPECT_MAT_NEAR(ned_hat, ned1, 1e-6);
}

TEST (GPS, initECEF)
{
    Simulator sim;
    std::string filename = "tmp.params.yaml";
    ofstream tmp_file(filename);
    YAML::Node node;
    node["ref_LLA"] = std::vector<double>{40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    node["gps_update_rate"] = 5;
    node["use_gps_truth"] = false;
    node["gps_horizontal_position_stdev"] = 1.0;
    node["gps_vertical_position_stdev"] = 3.0;
    node["gps_velocity_stdev"] = 0.1;
    tmp_file << node;
    tmp_file.close();

    sim.param_filename_ = filename;
    sim.init_gps();

    Vector3d to_center_of_earth = sim.x_e2n_.t().normalized();
    EXPECT_MAT_NEAR(-sim.x_e2n_.q().rotp(to_center_of_earth), e_z, 4e-3);
}
