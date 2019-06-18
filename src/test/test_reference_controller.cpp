#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/controller.h"

using namespace multirotor_sim;

class ReferenceControllerTest : public ::testing::Test
{
public:
  ReferenceControllerTest() :
    sim(false) {}

protected:

  void SetUp() override
  {
    filename = "tmp.params.yaml";
    std::ofstream tmp_file(filename);
    YAML::Node node = YAML::LoadFile(MULTIROTOR_SIM_DIR"/params/sim_params.yaml");
    node["tmax"] = 60;
    node["seed"] = 1;
    node["dt"] = 0.01;
    node["log_filename"] = "";
    node["imu_enabled"] = false;
    node["alt_enabled"] = false;
    node["mocap_enabled"] = false;
    node["vo_enabled"] = false;
    node["camera_enabled"] = false;
    node["gnss_enabled"] = false;
    node["raw_gnss_enabled"] = false;
    node["path_type"] = 0;
    node["waypoints"] = std::vector<double>
        {10, 0, -5, .705,
         10, 10, -4, 3.0,
        -10, 10, -5.5, -1.5,
        -10, -7, -5, .80,
         -8, -12, -4.5, -2.3,
          8, -5, -4, 1.7};
    node["control_type"] = 0;
    node["x0"] = std::vector<double>
       {0, 0, -5,
        1, 0, 0, 0,
        1, 0, 0,
        0, 0, 0};

    tmp_file << node;
    tmp_file.close();

    sim.load(filename);
  }
  Simulator sim;
  std::string filename;
};

TEST_F (ReferenceControllerTest, WaypointsSupplied)
{
  int prev_waypoint_id = -1;
  std::vector<double> waypoint_time;
  std::ofstream file("/tmp/ReferenceController.NonlinearController_Waypoints.log");

  ReferenceController cont;
  cont.load(filename);
  sim.use_custom_controller(&cont);
  sim.use_custom_trajectory(&cont);

  while(sim.run())
  {
      file.write((char*)&sim.t_, sizeof(double));
      file.write((char*)sim.state().arr.data(), sizeof(double) * State::SIZE);
      file.write((char*)sim.commanded_state().arr.data(), sizeof(double) * State::SIZE);
      if (cont.current_waypoint_id_ != prev_waypoint_id)
      {
          prev_waypoint_id = cont.current_waypoint_id_;
          waypoint_time.push_back(sim.t_);
      }
  }
  //  EXPECT_NEAR(waypoint_time[0], 0.01, 0.1);
//  EXPECT_NEAR(waypoint_time[1], 5.25, 0.1);
//  EXPECT_NEAR(waypoint_time[2], 16.45, 0.2);
//  EXPECT_NEAR(waypoint_time[3], 30.25, 0.5);
//  EXPECT_NEAR(waypoint_time[4], 44.32, 1.5);

  file.close();
}

TEST_F (ReferenceControllerTest, WaypointsInternal)
{
  int prev_waypoint_id = -1;
  std::vector<double> waypoint_time;
  std::ofstream file("ReferenceController.NonlinearController_Waypoints.log");

  while(sim.run())
  {
    file.write((char*)&sim.t_, sizeof(double));
    file.write((char*)sim.state().arr.data(), sizeof(double) * State::SIZE);
    file.write((char*)sim.commanded_state().arr.data(), sizeof(double) * State::SIZE);
    if (sim.ref_con_.current_waypoint_id_ != prev_waypoint_id)
    {
      prev_waypoint_id = sim.ref_con_.current_waypoint_id_;
      waypoint_time.push_back(sim.t_);
    }
  }
//  EXPECT_NEAR(waypoint_time[0], 0.01, 0.1);
//  EXPECT_NEAR(waypoint_time[1], 5.25, 0.1);
//  EXPECT_NEAR(waypoint_time[2], 16.45, 0.2);
//  EXPECT_NEAR(waypoint_time[3], 30.25, 0.5);
//  EXPECT_NEAR(waypoint_time[4], 44.32, 1.5);

  file.close();
}
