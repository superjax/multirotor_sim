#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/controller.h"

using namespace multirotor_sim;

class ReferenceControllerTest : public ::testing::Test
{
public:
  ReferenceControllerTest() :
    sim(cont, cont, false) {}

protected:

  void SetUp() override
  {
    std::string filename = "tmp.params.yaml";
    ofstream tmp_file(filename);
    YAML::Node node;
    node["tmax"] = 600.0;
    node["tmax"] = 60.0;
    node["seed"] = 1;
    node["dt"] = 0.01;
    node["log_filename"] = "";
    node["imu_enabled"] = false;
    node["alt_enabled"] = false;
    node["att_enabled"] = false;
    node["pos_enabled"] = false;
    node["vo_enabled"] = false;
    node["features_enabled"] = false;
    node["gnss_enabled"] = false;
    node["path_type"] = 0;
    node["waypoints"] = std::vector<double>
        {10, 0, -5, .705,
         10, 10, -4, 3.0,
        -10, 10, -5.5, -1.5,
        -10, -7, -5, .80,
         -8, -12, -4.5, -2.3,
          8, -5, -4, 1.7};
    node["control_type"] = 0;

    node["Kp"] = std::vector<double> {1, 1, 1};
    node["Kd"] = std::vector<double> {0, 0, 0};
    node["Kv"] = std::vector<double> {2, 2, 2};
    node["throttle_eq"] = 0.5;
    node["mass"] = 1.0;
    node["max_thrust"] = 19.6133;
    node["waypoint_threshold"] = 0.1;
    node["waypoint_velocity_threshold"] = 0.5;
    node["drag_constant"] = 0.1;

    node["sh_kv"] = 50;
    node["sh_ks"] = 0.1;
    node["roll_kp"] = 10.0;
    node["roll_ki"] = 0.0;
    node["roll_kd"] = 1.0;
    node["pitch_kp"] = 10.0;
    node["pitch_ki"] = 0.0;
    node["pitch_kd"] = 1.0;
    node["yaw_rate_kp"] = 1.0;
    node["yaw_rate_ki"] = 0.0;
    node["yaw_rate_kd"] = 0.0;
    node["max_roll"] = 1.0;
    node["max_pitch"] = 1.0;
    node["max_yaw_rate"] = 1.0;
    node["max_throttle"] = 1.0;
    node["max_vel"] = 5.0;
    node["max_tau_x"] = 1.0;
    node["max_tau_y"] = 1.0;
    node["max_tau_z"] = 1.0;

    node["angular_drag_constant"] = 0.01;
    node["RK4"] = true;
    node["p_b_u"] = std::vector<double>{0, 0, 0};
    node["q_b_u"] = std::vector<double>{1, 0, 0, 0};
    node["enable_wind"] = false;
    node["wind_init_stdev"] = 0.1;
    node["wind_walk_stdev"] = 0.1;
    node["x0"] = std::vector<double>
       {0, 0, -5,
        1, 0, 0, 0,
        1, 0, 0,
        0, 0, 0};
    node["inertia"] = std::vector<double>{0.1, 0.1, 0.1};

    tmp_file << node;
    tmp_file.close();

    sim.load(filename);
  }
  ReferenceController cont;
  Simulator sim;
};

TEST_F (ReferenceControllerTest, Waypoints)
{
  int prev_waypoint_id = cont.current_waypoint_id_;
  std::vector<double> waypoint_time;
  ofstream file("ReferenceController.NonlinearController_Waypoints.log");
  while(sim.run())
  {
    file.write((char*)&sim.t_, sizeof(double));
    file.write((char*)sim.dyn_.get_state().arr.data(), sizeof(double) * State::SIZE);
    file.write((char*)sim.traj_.getCommandedState(sim.t_).arr.data(), sizeof(double) * State::SIZE);
    if (cont.current_waypoint_id_ != prev_waypoint_id)
    {
      prev_waypoint_id = cont.current_waypoint_id_;
      waypoint_time.push_back(sim.t_);
    }
  }
  EXPECT_NEAR(waypoint_time[0], 28.12, 1e-8);
  EXPECT_NEAR(waypoint_time[1], 57.14, 1e-8);

  file.close();

}
