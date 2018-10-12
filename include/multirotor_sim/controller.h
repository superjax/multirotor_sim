#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/StdVector>

#include "utils.h"
#include "quat.h"
#include "support.h"
#include "dynamics.h"

using namespace quat;
using namespace Eigen;

namespace controller
{

class Controller
{

public:

  Controller();
  
  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ,
    PSI
  };
  
  typedef struct
  {
    double roll;
    double pitch;
    double yaw_rate;
    double throttle;
    double vel;
  } max_t;
  
  typedef struct
  {
    double t;
    double pn;
    double pe;
    double pd;
  
    double phi;
    double theta;
    double psi;
  
    double u;
    double v;
    double w;
  
    double p;
    double q;
    double r;
  
    double throttle;
  } state_t;
  
  class PID
    {
    public:
      PID();
      void init(float kp, float ki, float kd, float max, float min, float tau);
      float run(float dt, float x, float x_c, bool update_integrator);
      float run(float dt, float x, float x_c, bool update_integrator, float xdot);

      float kp_;
      float ki_;
      float kd_;
  
      float max_;
  
      float integrator_;
      float differentiator_;
      float prev_x_;
      float tau_;
  };
  PID roll_;
  PID pitch_;
  PID yaw_rate_;

  // Parameters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double drag_constant_;
  double waypoint_threshold_;
  double waypoint_velocity_threshold_;
  double vmag_;
  int path_type_;
  double traj_heading_walk_;
  double traj_heading_straight_gain_;
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> udist_;

  // Circular Trajectory Parameters
  double traj_delta_north_;
  double traj_delta_east_;
  double traj_delta_alt_;
  double traj_delta_yaw_;
  double traj_nom_north_;
  double traj_nom_east_;
  double traj_nom_alt_;
  double traj_nom_yaw_;
  double traj_north_freq_;
  double traj_east_freq_;
  double traj_alt_freq_;
  double traj_yaw_freq_;

  // Controller Gains
  Eigen::Matrix3d K_p_; // position
  Eigen::Matrix3d K_v_; // velocity
  Eigen::Matrix3d K_d_; // disturbance acceleration
  Eigen::MatrixXd waypoints_;
  int current_waypoint_id_;

  // Memory for sharing information between functions
  bool initialized_;
  state_t xhat_ = {}; // estimate
  state_t xc_ = {}; // command
  max_t max_ = {};
  double prev_time_;
  uint8_t control_mode_;
  Eigen::Vector3d dhat_; // disturbance acceleration

public:
  // Functions
  void load(const std::string filename);
  void updateWaypointManager();
  void updateTrajectoryManager();
  void computeControl(const dynamics::xVector &x, const double t, dynamics::commandVector& u); 
  
  inline state_t getCommandedState() const { return xc_; } 
  const Eigen::MatrixXd get_waypoints() const { return waypoints_; }
};

}
