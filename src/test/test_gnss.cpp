#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/controller.h"
#include "multirotor_sim/utils.h"
#include "multirotor_sim/wsg84.h"
#include "multirotor_sim/test_common.h"

using namespace multirotor_sim;

TEST (Gnss, lla2ecef)
{
    Vector3d lla = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511}; // BYU Campus
    Vector3d ecef_known = {-1798810.23, -4532232.54, 4099784.74};

    Vector3d ecef_calc = WSG84::lla2ecef(lla);

    ASSERT_MAT_NEAR(ecef_known, ecef_calc, 1e-2);
}

TEST (Gnss, ecef2lla)
{
    Vector3d ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vector3d lla_known = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.998309};

    Vector3d lla_calc = WSG84::ecef2lla(ecef);

    ASSERT_MAT_NEAR(lla_known, lla_calc, 1e-6);
}

TEST (Gnss, ecef2lla2ecef)
{
    Vector3d ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vector3d lla_known = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.998309};

    Vector3d lla_calc = WSG84::ecef2lla(ecef);
    Vector3d ecef_calc = WSG84::lla2ecef(lla_calc);

    ASSERT_MAT_NEAR(ecef_calc, ecef, 1e-6);
}

TEST (Gnss, x_ned2ecef)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d ned1 = {-54.976484, 1.276565, 0.000237};
    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);

    xform::Xformd x_e2n = WSG84::x_ecef2ned(ecef0);
    Vector3d ecef_hat = x_e2n.transforma(ned1);
    Vector3d ned1_hat = x_e2n.transformp(ecef1);

    EXPECT_MAT_NEAR(ecef_hat, ecef1, 1e-6);
    EXPECT_MAT_NEAR(ned1_hat, ned1, 1e-3);
}

TEST (Gnss, ecef2ned_check_axes)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);
    xform::Xformd x_e2n = WSG84::x_ecef2ned(ecef0);

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

TEST (Gnss, ned2ecef)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d ned1 = {-54.976484, 1.276565, 0.000237};
    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);

    xform::Xformd x_e2n = WSG84::x_ecef2ned(ecef0);
    Vector3d ecef_hat = WSG84::ned2ecef(x_e2n, ned1);
    Vector3d ned1_hat = WSG84::ecef2ned(x_e2n, ecef1);

    EXPECT_MAT_NEAR(ecef_hat, ecef1, 1e-6);
    EXPECT_MAT_NEAR(ned1_hat, ned1, 1e-6);
}

TEST (Gnss, lla2ned)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ned1 = {-54.976484, 1.276565, 0.000237};

    Vector3d ned1_hat = WSG84::lla2ned(lla0, lla1);
    Vector3d lla1_hat = WSG84::ned2lla(lla0, ned1);

    EXPECT_MAT_NEAR(lla1_hat, lla1, 1e-6);
    EXPECT_MAT_NEAR(ned1_hat, ned1, 1e-6);
}

class GnssTestEstimator : public EstimatorBase
{
public:
    void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) override {}
    void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) override {}
    void posCallback(const double& t, const Vector3d& z, const Matrix3d& R) override {}
    void attCallback(const double& t, const Quatd& z, const Matrix3d& R) override {}
    void voCallback(const double& t, const Xformd& z, const Matrix6d& R) override {}
    void featCallback(const double& t, const Vector2d& z, const Matrix2d& R, int id, double depth) override {}
    void rawGnssCallback(const GTime& t, const Vector3d& z, const Matrix3d& R, int id) override {}
    void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) override
    {
        call_count++;
        z_last = z;
    }

    int call_count = 0;
    Vector6d z_last;
};

class GnssTest : public ::testing::Test {
protected:
    GnssTest() :
        sim(cont, cont)
    {}
    void SetUp() override
    {
        std::string filename = "tmp.params.yaml";
        ofstream tmp_file(filename);
        YAML::Node node;
        node["ref_LLA"] = std::vector<double>{40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
        node["gnss_update_rate"] = 5;
        node["gnss_enabled"] = true;
        node["use_gnss_truth"] = false;
        node["gnss_horizontal_position_stdev"] = 1.0;
        node["gnss_vertical_position_stdev"] = 3.0;
        node["gnss_velocity_stdev"] = 0.1;
        tmp_file << node;
        tmp_file.close();

        sim.param_filename_ = filename;
        sim.init_gnss();
        sim.register_estimator(&est);
    }

    ReferenceController cont;
    Simulator sim;
    GnssTestEstimator est;
};

TEST_F (GnssTest, initECEF)
{
    Vector3d to_center_of_earth = sim.x_e2n_.t().normalized();
    EXPECT_MAT_NEAR(-sim.x_e2n_.q().rotp(to_center_of_earth), e_z, 4e-3);
}


TEST_F (GnssTest, MeasurementUpdateRate)
{
    sim.update_gnss_meas();
    ASSERT_EQ(est.call_count, 0);

    State x;
    x.p << 1000, 0, 0;
    sim.dyn_.set_state(x);
    sim.t_ = 0.2;

    sim.update_gnss_meas();
    ASSERT_EQ(est.call_count, 1);

    sim.update_gnss_meas();
    ASSERT_EQ(est.call_count, 1);
}

TEST_F (GnssTest, MeasurementPosition)
{

    State x;
    x.p << 1000, 0, 0;
    sim.dyn_.set_state(x);
    sim.t_ = 0.2;

    sim.update_gnss_meas();

    Vector3d lla0 = (Vector3d() << 40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309).finished();
    Vector3d ecef0 = WSG84::lla2ecef(lla0);
    xform::Xformd x_e2n = WSG84::x_ecef2ned(ecef0);
    Vector3d ned1 = x.p;
    Vector3d ecef1 = WSG84::ned2ecef(x_e2n, ned1);

    EXPECT_NEAR((ecef1 - ecef0).norm(), 1000, 1e-6);
    ASSERT_MAT_NEAR(est.z_last.segment<3>(0), ecef1, 10.0);
    ASSERT_MAT_NEAR(est.z_last.segment<3>(3), Vector3d::Constant(0.0), 1.0);
}

TEST_F (GnssTest, MeasurementVelocity)
{
    State x;
    x.v << 0, 0, -10;
    sim.dyn_.set_state(x);
    sim.t_ = 0.2;

    sim.update_gnss_meas();

    Vector3d lla0 = (Vector3d() << 40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309).finished();
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d vel_dir_ECEF = ecef0.normalized() * 10;

    ASSERT_MAT_NEAR(est.z_last.segment<3>(3), vel_dir_ECEF, 1.0);
}

