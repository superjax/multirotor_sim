# Multirotor Sim
[![Build Status](https://travis-ci.com/superjax/multirotor_sim.svg?branch=master)](https://travis-ci.com/superjax/multirotor_sim)

This package is a pure C++ library which simulates a multirotor flying through some environment.

### Simulated Sensors:
 * 6-DOF Accelerometer with constant bias
 * Altimeter
 * GPS solution
 * Raw GPS Measurements (pseudorange, pseudorange rate, carrier phase)
 * Visual Odometry
 * Pixel Measurements to tracked features

# Installing and Building
This package depends on CMake, Eigen and Yaml-CPP.  If you want to run the unit tests, you'll also need GoogleTest

### Installing Eigen and Yaml-CPP
``` bash
sudo apt install -y cmake libeigen3-dev libyaml-cpp-dev
```

### Installing googletest
``` bash
sudo apt install -y libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```

### Building and Running Unit Tests
``` bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j -l
./multirotor_sim_test
```



# Default Behavior
Here is a simple example of how to use the simulator.
``` C++
#include "multirotor_sim/simulator.h"
using namespace multirotor_sim;

int main
{
    Simulator sim;
    sim.load("../params/sim_params.yaml");

    while (sim.run())
    {
        ...
    }
}
```

This by default runs a reference nonlinear controller and follows the waypoints specified in the `sim_params.yaml` file.

The constructor to the `Simulator` object takes two optional variables.  The first is whether or not to render a progress bar during simulation (default=`false`) and the second is a random seed.  If the seed is `0`, then a seed is generated using the current clock (default=`0`).

`sim.run()` will return true until the time variable reaches `tmax` specified in the yaml file, and each call will integrate the dynamics by the `dt` parameter.

The state of the simulator at each step is accessed through the `sim.state()` function, and various other data access mechanisms are given in the `Simulator` definition.

# Configuration
Configuration of the simulator is done in the provided `.yaml` file.  Configuration options include the rate of the dynamic integration, which sensors are enabled, etc...

# Using a Custom Controller, Estimator, and Trajectory
The simulator also supports the use of a custom trajectory, controller and estimator.  All you have to do is inherit from the base class and implement the approprate functions.

## Custom Controller
The interface to the controller object is given by the virtual `ControllerBase` class.

``` C++
class ControllerBase
{
public:
  // t - current time (seconds)
  // x - current state
  // x_c - desired state
  // u - output [F, tau_x, tau_y, tau_z].T
  virtual void computeControl(const double& t, const State& x, const State& x_c, Vector4d& u) = 0;
};
```

To use this controller, just provide a pointer to it in the `use_custom_controller()` function.


``` C++
#include "multirotor_sim/simulator.h"
using namespace multirotor_sim;

int main
{
    Simulator sim;
    sim.load("../params/sim_params.yaml");

    CustomController controller;
    sim.use_custom_controller(&controller);

    while (sim.run())
    {
        ...
    }
}
```

A custom trajectory is used in the same way, but must inherit from the `TrajectoryBase` class instead.

``` C++
class TrajectoryBase
{
public:
  // t - current time (seconds)
  // (return) commanded state
  virtual const State& getCommandedState(const double& t) = 0;
};
```


## Custom State Estimator
The interface to the estimator object is given by the virtual `EstimatorBase` class.

``` C++
class EstimatorBase
{
public:
    // t - current time (seconds)
    // z - imu measurement [acc, gyro]
    // R - imu covariance
    virtual void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    virtual void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void mocapCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void voCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void imageCallback(const double& t, const Image& z, const Matrix2d& R) {}

    // t - current time (seconds)
    // z - gnss measurement [p_{b/ECEF}^ECEF, v_{b/ECEF}^ECEF]
    // R - gnss covariance
    virtual void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    // t - Time of measurement (GPS Time)
    // z - gnss measurement [rho(m), rhodot(m/s), l(cycles)]
    // R - gnss covariance
    // sat - Satellite object related to this measurement
    virtual void rawGnssCallback(const GTime& t, const Vector3d& z, const Matrix3d& R, Satellite& sat) {}
};
```

To implement a custom estimator, inherit from `EstimatorBase` and override whatever callbacks you want to use.  You do not need to implement all the callbacks if you don't want to.  Once you have a custom estimator, just provide the simulator object a pointer to the estimator.

``` C++
#include "multirotor_sim/simulator.h"
using namespace multirotor_sim;

int main
{
    Simulator sim;
    sim.load("../params/sim_params.yaml");

    CustomEstimator estimator;
    sim.register_estimator(&estimator);

    while (sim.run())
    {
        ...
    }
}
```

You can register as many estimators as you want, and they will all be given exactly the same data.  Sensor measurements are generated at the closest simulation step to the specified update rate in the simulation configuration `yaml` file.

# State and ErrorState Objects
One potentially confusing thing is the way that the `State` object and `ErrorState` object are defined.

Both of these objects have an underlying Eigen Array.  The `State` has a 17x1 array, while the `ErrorState` is a 16x1.  This array is then [Map](https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html)ped. with several small accessors which provide convenient access to the parts of the state.  For example, if I wanted to access the position state of some `State x;`, I would just type

``` C++
State x;
Vector3d position = x.p;
```

The attitude is represented as a quaternion, accessed through `State.q`, and the homogeneous transform (the combination of position and attitude) is given by `State.X`.  

The `State` and `ErrorState` objects have operator overloads to allow things like addition and subtraction (similar to the $\boxminus$ operators defined in [Hertzberg et. al](https://arxiv.org/abs/1107.1119)).
```
State x1;
ErrorState dx;

State x2 = x1 + dx;

ErrorState dx2 = x2 - x1;
```

If you want to access the underlying array directly, you can always access it with the `arr()` function;
