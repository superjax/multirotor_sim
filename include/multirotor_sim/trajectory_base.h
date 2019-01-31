#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

class TrajectoryBase
{
public:
  // t - current time (seconds)
  // (return) commanded state
  virtual const State& getCommandedState(const double& t) = 0;
};

}
