#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

class TrajectoryBase
{
public:
  // t - current time (seconds)
  // (return) x_c commanded state
  // (return) u_r reference input
  virtual void getCommandedState(const double& t, State& x_c, Vector4d& u_r) = 0;
};

}
