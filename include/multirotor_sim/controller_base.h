#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

class ControllerBase
{
public:

  // t - current time (seconds)
  // x - current state
  // xc - desired state
  // ur - reference input
  // u - output [F, tau_x, tau_y, tau_z].T
  virtual void computeControl(const double& t, const State& x, const State& xc,
                              const Eigen::Vector4d& ur, Eigen::Vector4d& u) = 0;
};

}
