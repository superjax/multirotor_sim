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

class RawGpsTest : public ::testing::Test {
protected:
    RawGpsTest() :
        sim(cont, cont, false, 1)
    {}

    void SetUp() override
    {
        std::string filename = "tmp.params.yaml";
        ofstream tmp_file(filename);
        YAML::Node node;
        node["ref_LLA"] = std::vector<double>{40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
        node["gnss_update_rate"] = 5;
        node["use_raw_gnss_truth"] = false;
        node["pseudorange_stdev"] = 1.0;
        node["pseudorange_rate_stdev"] = 0.1;
        node["carrier_phase_stdev"] = 0.1;
        node["ephemeris_filename"] = "../sample/eph.dat";
        tmp_file << node;
        tmp_file.close();

        cont.load("../params/sim_params.yaml");
        sim.param_filename_ = filename;
        sim.init_raw_gnss();
    }
    ReferenceController cont;
    Simulator sim;
};

TEST_F (RawGpsTest, PseudorangeFromNED)
{
    EXPECT_EQ(sim.satellites_.size(), 15);
}
