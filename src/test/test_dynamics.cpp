#include <gtest/gtest.h>
#include <fstream>
#include <random>

#include "multirotor_sim/dynamics.h"
#include "multirotor_sim/utils.h"

TEST (Dynamics, Propagate)
{
  std::string filename = "tmp.params.yaml";
  std::ofstream tmp_file(filename);
  YAML::Node node;
  node["throttle_eq"] = 0.5;
  node["mass"] = 1.0;
  node["seed"] = 1;
  node["max_thrust"] = 19.6133;
  node["waypoint_threshold"] = 0.1;
  node["waypoint_velocity_threshold"] = 0.5;
  node["drag_constant"] = 0.1;

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

  multirotor_sim::Dynamics dyn;
  dyn.load(filename);


  std::ofstream file("Dynamics.Propagate.log");
  double t;
  double dt = 0.01;
  Vector4d u;
  u.setZero();
  std::normal_distribution<double> dist;
  std::default_random_engine gen;
  u(0) = 0.5;

  for (int i = 0; i < 50; i++)
  {
    t = dt * i;
    Vector4d noise;
    random_normal_vec(noise, 0.1, dist, gen);
    u += dt * noise;
    dyn.run(t, u);
    file.write((char*)&t, sizeof(double));
    file.write((char*)dyn.get_state().arr.data(), sizeof(double) * multirotor_sim::State::SIZE);
    EXPECT_NEAR(dyn.get_state().q.arr_.norm(), 1.0, 1e-14);
  }

  file.close();

}
