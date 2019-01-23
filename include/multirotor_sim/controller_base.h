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
    // x_c - desired state
    // u - output [F, tau_x, tau_y, tau_z].T
    virtual void computeControl(const double& t, const State& x, const State& x_c, Vector4d& u) = 0;
};

}
