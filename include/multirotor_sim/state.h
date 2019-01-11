#pragma once

#include <Eigen/Core>
#include <geometry/xform.h>

using namespace Eigen;
using namespace xform;
using namespace quat;

struct ErrorState
{
    enum
    {
        SIZE = 12
    };


    Matrix<double, SIZE, 1> arr;
    Map<Vector6d> X;
    Map<Vector3d> p;
    Map<Vector3d> q;
    Map<Vector3d> v;
    Map<Vector3d> w;

    ErrorState() :
        X(arr.data()),
        p(arr.data()),
        q(arr.data()+3),
        v(arr.data()+6),
        w(arr.data()+9)
    {}

    ErrorState(const ErrorState& obj) :
        X(arr.data()),
        p(arr.data()),
        q(arr.data()+3),
        v(arr.data()+6),
        w(arr.data()+9)
    {
        arr = obj.arr;
    }

    ErrorState& operator= (const ErrorState& obj)
    {
        arr = obj.arr;
        return *this;
    }
};

struct State
{
    enum
    {
        SIZE = 13
    };


    Matrix<double, SIZE, 1> arr;
    Xformd X;
    Map<Vector3d> p;
    Quatd q;
    Map<Vector3d> v;
    Map<Vector3d> w;

    State() :
        X(arr.data()),
        p(arr.data()),
        q(arr.data()+3),
        v(arr.data()+7),
        w(arr.data()+10)
    {
        arr.setZero();
        q = Quatd::Identity();
    }

    State(const State& x) :
        X(arr.data()),
        p(arr.data()),
        q(arr.data()+3),
        v(arr.data()+7),
        w(arr.data()+10)
    {
        arr = x.arr;
    }

    State& operator= (const State& obj)
    {

        arr = obj.arr;
        return *this;
    }

    State operator+(const ErrorState& dx) const
    {
        State xp;
        xp.X = X + dx.X;
        xp.v = v + dx.v;
        xp.w = w + dx.w;
        return xp;
    }

    State& operator+=(const ErrorState& dx)
    {
        X = X + dx.X;
        v = v + dx.v;
        w = w + dx.w;
        return *this;
    }

    ErrorState operator-(const State& x) const
    {
        ErrorState dx;
        dx.X = X - x.X;
        dx.v = v - x.v;
        dx.w = w - x.w;
        return dx;
    }
};

struct Input
{
    enum
    {
        SIZE = 4
    };
    Matrix<double, SIZE, 1> arr;
    double& T;
    Map<Vector3d> tau;

    Input() :
        T(*arr.data()),
        tau(arr.data()+3)
    {}
};

struct IMU
{
    enum
    {
        SIZE = 6
    };
    Matrix<double, SIZE, 1> arr;
    Map<Vector3d> acc;
    Map<Vector3d> gyro;

    IMU() :
        acc(arr.data()),
        gyro(arr.data()+3)
    {}
};
