#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/controller.h"
#include "multirotor_sim/utils.h"
#include "multirotor_sim/wsg84.h"
#include "multirotor_sim/raw_gnss.h"
#include "multirotor_sim/test_common.h"

using namespace Eigen;
using namespace std;
using namespace multirotor_sim;

class RawGnssTestEstimator : public EstimatorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) override {}
    void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) override {}
    void mocapCallback(const double& t, const Xformd& z, const Matrix6d& R) override {}
    void voCallback(const double& t, const Xformd& z, const Matrix6d& R) override {}
    void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) override {}
    void rawGnssCallback(const GTime& t, const Vector3d& z, const Matrix3d& R, Satellite& sat) override
    {
        time_last = t;
        call_count++;
        z_last.push_back(z);
        if (((z.array()) != z.array()).any())
        {
            int debug = 1;
        }
    }

    GTime time_last;
    int call_count = 0;
    std::vector<Vector3d, aligned_allocator<Vector3d>> z_last;
};


class RawGpsTest : public ::testing::Test {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    RawGpsTest() :
        sim(false, 1)
    {}

    void SetUp() override
    {
        std::string filename = "tmp.params.yaml";
        ofstream tmp_file(filename);
        YAML::Node node;
        node["ref_LLA"] = std::vector<double>{40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
        node["gnss_update_rate"] = 5;
        node["use_raw_gnss_truth"] = false;
        node["pseudorange_stdev"] = 3.0;
        node["pseudorange_rate_stdev"] = 0.1;
        node["carrier_phase_stdev"] = 0.01;
        node["ephemeris_filename"] = "../sample/eph.dat";
        node["start_time_week"] = 2026;
        node["start_time_tow_sec"] = 165029;
        node["clock_init_stdev"] = 1e-4;
        node["clock_walk_stdev"] = 1e-7;
        tmp_file << node;
        tmp_file.close();

        cont.load("../params/sim_params.yaml");
        sim.param_filename_ = filename;
        sim.init_raw_gnss();
        sim.register_estimator(&est);
        sim.t_ = 0.0;
    }
    ReferenceController cont;
    Simulator sim;
    RawGnssTestEstimator est;
};


TEST_F (RawGpsTest, PseudorangeFromNED)
{
    EXPECT_EQ(sim.satellites_.size(), 15);
}

TEST_F (RawGpsTest, MeasurementUpdateRate)
{
    sim.update_raw_gnss_meas();
    ASSERT_EQ(est.call_count, 0);

    State x;
    x.p << 1000, 0, 0;
    sim.dyn_.set_state(x);
    sim.t_ = 0.3;

    sim.update_raw_gnss_meas();
    ASSERT_EQ(est.call_count, 15);

    for (int i = 0; i < 15; i++)
    {
        EXPECT_TRUE((est.z_last[i].array() == est.z_last[i].array()).all());
    }

    sim.update_raw_gnss_meas();
    ASSERT_EQ(est.call_count, 15);
}

TEST_F (RawGpsTest, MeasurementIsCloseToTruth)
{
    State x;
    x.p << 1000, 0, 0;
    sim.dyn_.set_state(x);
    sim.t_ = 0.3;
    sim.update_raw_gnss_meas();

    GTime t = sim.t_ + sim.start_time_;
    Vector3d pos_ecef = WSG84::ned2ecef(sim.X_e2n_, x.p);
    Vector3d vel_ned = sim.dyn_.get_state().q.rota(sim.dyn_.get_state().v);
    Vector3d vel_ecef = sim.X_e2n_.q().rota(vel_ned);
    Vector3d z_true;
    for (int i = 0; i < 15; i++)
    {
        sim.satellites_[i].computeMeasurement(t, pos_ecef, vel_ecef, Vector2d{sim.clock_bias_, sim.clock_bias_rate_}, z_true);
        EXPECT_NEAR(z_true[0], est.z_last[i][0], 4.0);
        EXPECT_NEAR(z_true[1], est.z_last[i][1], 0.3);
        EXPECT_NEAR(z_true[2], est.z_last[i][2], 100);
    }
}

TEST_F (RawGpsTest, LeastSquaresPositioningPseudoranges)
{
    State x;
    x.p << 1000, 0, 0;
    sim.dyn_.set_state(x);
    sim.t_ = 0.3;
    sim.update_raw_gnss_meas();

    GTime t = sim.t_ + sim.start_time_;
    Vector3d vel_ned = sim.dyn_.get_state().q.rota(sim.dyn_.get_state().v);
    Vector3d vel_ecef = sim.X_e2n_.q().rota(vel_ned);

    Vector3d xhat = Vector3d::Zero();
    Vector3d xtrue = WSG84::ned2ecef(sim.X_e2n_, x.p);

    Matrix<double, 15, 4> A;
    Matrix<double, 15, 1> b;
    Matrix<double, 4, 1> dx;

    int iter = 0;
    do
    {
        iter++;
        for (int i = 0; i < 15; i++)
        {
            Vector3d sat_pos, sat_vel;
            Vector2d sat_clk_bias;
            sim.satellites_[i].computePositionVelocityClock(t, sat_pos, sat_vel, sat_clk_bias);

            Vector3d z ;
            sim.satellites_[i].computeMeasurement(t, xhat, vel_ecef, Vector2d::Zero(), z);
            b(i) = est.z_last[i](0) - z(0);

            A.block<1,3>(i,0) = (xhat - sat_pos).normalized().transpose();
            A(i,3) = Satellite::C_LIGHT;
        }

        ColPivHouseholderQR<Matrix<double, 15, 4>> solver(A);
        dx = solver.solve(b);

        xhat += dx.segment<3>(0);
        t += dx(3);
    } while (dx.norm() > 1e-4);

//    cout << (xhat - xtrue).transpose() << endl;
    Vector3d xhat_ned = WSG84::ecef2ned(sim.X_e2n_, xhat);
    Vector3d xtrue_ned = WSG84::ecef2ned(sim.X_e2n_, xtrue);
//    cout << "xhat:\n" << xhat_ned.transpose() <<
//            "\nx:\n" << xtrue_ned.transpose() << endl;

    EXPECT_NEAR(xhat_ned.x(), xtrue_ned.x(), 2.0);
    EXPECT_NEAR(xhat_ned.y(), xtrue_ned.y(), 2.0);
    EXPECT_NEAR(xhat_ned.z(), xtrue_ned.z(), 3.0);
    EXPECT_LT(iter, 6);
}
