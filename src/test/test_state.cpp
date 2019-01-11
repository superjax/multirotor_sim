#include <gtest/gtest.h>

#include "multirotor_sim/state.h"
#include "multirotor_sim/test_common.h"

TEST (ErrorState, Construct)
{
    ErrorState dx;
}

TEST (ErrorState, Copy)
{
    ErrorState dx1;
    ErrorState dx2;

    dx1.arr.setZero();
    dx2 = dx1;

    EXPECT_MAT_EQ(dx2.arr, dx1.arr);

    dx2.p.setConstant(1.0);
    for (int i = 0; i < ErrorState::SIZE; i++)
    {
        EXPECT_FLOAT_EQ(dx1.arr(i), 0);
    }
    for (int i = 0; i < 3; i++)
    {
        EXPECT_FLOAT_EQ(dx2.arr(i), 1.0);
    }

}

TEST (ErrorState, CopyEquals)
{
    ErrorState dx1;
    dx1.arr.setZero();

    ErrorState dx2 = dx1;

    EXPECT_MAT_EQ(dx2.arr, dx1.arr);

    dx2.p.setConstant(1.0);
    for (int i = 0; i < ErrorState::SIZE; i++)
    {
        EXPECT_FLOAT_EQ(dx1.arr(i), 0);
    }
    for (int i = 0; i < 3; i++)
    {
        EXPECT_FLOAT_EQ(dx2.arr(i), 1.0);
    }

}

TEST (ErrorState, CopyConstruct)
{
    ErrorState dx1;
    dx1.arr.setZero();

    ErrorState dx2(dx1);
    EXPECT_MAT_EQ(dx2.arr, dx1.arr);

    dx2.p.setConstant(1.0);
    for (int i = 0; i < ErrorState::SIZE; i++)
    {
        EXPECT_FLOAT_EQ(dx1.arr(i), 0);
    }
    for (int i = 0; i < 3; i++)
    {
        EXPECT_FLOAT_EQ(dx2.arr(i), 1.0);
    }
}

TEST (State, Construct)
{
    State x;
}

TEST (State, Copy)
{
    State x1;
    State x2;

    x1.X = Xformd::Identity();
    x2 = x1;

    EXPECT_MAT_EQ(x2.arr, x1.arr);

    x2.p.setConstant(1.0);
    for (int i = 0; i < ErrorState::SIZE; i++)
    {
        if (i == 3)
            EXPECT_FLOAT_EQ(x1.arr(i), 1.0); // quaternion w
        else
            EXPECT_FLOAT_EQ(x1.arr(i), 0);
    }
    for (int i = 0; i < 3; i++)
    {
        EXPECT_FLOAT_EQ(x2.arr(i), 1.0);
    }
}

TEST (State, Plus)
{
    State x1, x2;
    x1.X = Xformd::Identity();
    x2.X = Xformd::Identity();

    ErrorState dx;
    dx.arr.setRandom();

    x2 = x1 + dx;

    EXPECT_MAT_EQ(x2.X.elements(), (x1.X + dx.X).elements());
    EXPECT_MAT_EQ(x2.v, x1.v + dx.v);
    EXPECT_MAT_EQ(x2.w, x1.w + dx.w);
}

TEST (State, Minus)
{
    State x1, x2;
    x1.arr.setRandom();
    x2.arr.setRandom();
    x1.X = Xformd::Random();
    x2.X = Xformd::Random();

    ErrorState dx = x1 - x2;

    EXPECT_MAT_EQ(dx.X, x1.X - x2.X);
    EXPECT_MAT_EQ(dx.v, x1.v - x2.v);
    EXPECT_MAT_EQ(dx.w, x1.w - x2.w);
}

TEST (State, BoxplusRules)
{
    State x1, x2;
    x1.arr.setRandom();
    x2.arr.setRandom();
    x1.X = Xformd::Random();
    x2.X = Xformd::Random();

    ErrorState dx1, dx2;
    dx1.arr.setRandom();
    dx2.arr.setRandom();

    ErrorState zeros;
    zeros.arr.setZero();


    // Rules from Hertzberg et al. (11a-11d)
    EXPECT_MAT_EQ((x1 + zeros).arr, x1.arr);
    EXPECT_MAT_EQ((x1 + (x2 - x1)).arr, x2.arr);
    EXPECT_MAT_EQ(((x1 + dx1) - x1).arr, dx1.arr);
//    EXPECT_LE(((x1 + dx1) - (x1 + dx2)).arr.norm(), (dx1.arr - dx2.arr).norm());
}
