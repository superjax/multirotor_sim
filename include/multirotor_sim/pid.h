// General PID controller
#pragma once

template<typename T>
struct max_struct
{
  T roll;
  T pitch;
  T yaw_rate;
  T throttle;
  T vel;
};

template<typename T>
struct state_struct
{
  T t;
  T pn;
  T pe;
  T pd;

  T phi;
  T theta;
  T psi;

  T u;
  T v;
  T w;

  T p;
  T q;
  T r;

  T throttle;
};

typedef max_struct<double> max_t;
typedef state_struct<double> state_t;

template<typename T>
class PID
{
public:
  PID() :
    kp_(0.0),
    ki_(0.0),
    kd_(0.0),
    max_(1.0),
    integrator_(0.0),
    differentiator_(0.0),
    prev_x_(0.0),
    tau_(0.05)
  {}

  void init(T kp, T ki, T kd, T max, T min, T tau)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_ = max;
    tau_ = tau;
  }

  T run(T dt, T x, T x_c, bool update_integrator)
  {
    T xdot;
    if (dt > (T)0.0001)
    {
      // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
      // The dirty derivative is a sort of low-pass filtered version of the derivative.
      //// (Include reference to Dr. Beard's notes here)
      differentiator_ = ((T)2.0 * tau_ - dt) / ((T)2.0 * tau_ + dt) * differentiator_
          + (T)2.0 / ((T)2.0 * tau_ + dt) * (x - prev_x_);
      xdot = differentiator_;
    }
    else
    {
      xdot = (T)0.0;
    }
    prev_x_ = x;

    return run(dt, x, x_c, update_integrator, xdot);
  }

  T run(T dt, T x, T x_c, bool update_integrator, T xdot)
  {
    // Calculate Error
    T error = x_c - x;

    // Initialize Terms
    T p_term = error * kp_;
    T i_term = (T)0.0;
    T d_term = (T)0.0;

    // If there is a derivative term
    if (kd_ > (T)0.0)
    {
      d_term = kd_ * xdot;
    }

    //If there is an integrator term and we are updating integrators
    if ((ki_ > (T)0.0) && update_integrator)
    {
      // integrate
      integrator_ += error * dt;
      // calculate I term
      i_term = ki_ * integrator_;
    }

    // sum three terms
    T u = p_term - d_term + i_term;

    // Integrator anti-windup
    T u_sat = (u > max_) ? max_ : (u < (T)-1.0 * max_) ? (T)-1.0 * max_ : u;
    if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > (T)0.0)
      integrator_ = (u_sat - p_term + d_term)/ki_;

    // Set output
    return u_sat;
  }

  T kp_;
  T ki_;
  T kd_;

  T max_;

  T integrator_;
  T differentiator_;
  T prev_x_;
  T tau_;
};


