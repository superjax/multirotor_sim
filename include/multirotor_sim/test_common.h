#pragma once
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "multirotor_sim/ephemeris.h"
#include "multirotor_sim/gtime.h"
#include "multirotor_sim/datetime.h"
#include "multirotor_sim/test_common.h"

static const double RAD2DEG = 180.0 / M_PI;
static const double DEG2RAD = M_PI / 180.0;


#define ASSERT_MAT_NEAR(mat1, mat2, tol)\
do{  \
    ASSERT_EQ((mat1).rows(), (mat2).rows()); \
    ASSERT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            ASSERT_NEAR((mat1)(r,c), (mat2)(r,c), tol); \
        }\
    } \
} while(0)

#define EXPECT_MAT_NEAR(mat1, mat2, tol)\
do{  \
    EXPECT_EQ((mat1).rows(), (mat2).rows()); \
    EXPECT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            EXPECT_NEAR((mat1)(r,c), (mat2)(r,c), tol); \
        }\
    } \
} while(0)


#define ASSERT_MAT_EQ(mat1, mat2)\
do{  \
    ASSERT_EQ((mat1).rows(), (mat2).rows()); \
    ASSERT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            ASSERT_FLOAT_EQ((mat1)(r,c), (mat2)(r,c)); \
        }\
    } \
} while(0)

#define EXPECT_MAT_EQ(mat1, mat2)\
do{  \
    EXPECT_EQ((mat1).rows(), (mat2).rows()); \
    EXPECT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            EXPECT_FLOAT_EQ((mat1)(r,c), (mat2)(r,c)); \
        }\
    } \
} while(0)

#define EXPECT_MAT_NE(mat1, mat2)\
do{  \
    EXPECT_EQ((mat1).rows(), (mat2).rows()); \
    EXPECT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            EXPECT_NE((mat1)(r,c), (mat2)(r,c)); \
        }\
    }\
} while(0)


#define ASSERT_MAT_NE(mat1, mat2)\
do{  \
    ASSERT_EQ((mat1).rows(), (mat2).rows()); \
    ASSERT_EQ((mat1).cols(), (mat2).cols()); \
    for (int r = 0; r < (mat1).rows(); r++) { \
        for (int c = 0; c < (mat1).cols(); c++) {\
            ASSERT_NE((mat1)(r,c), (mat2)(r,c)); \
        }\
    } \
} while(0)

void eph2pos(const GTime& t, const Ephemeris* eph, Eigen::Vector3d& pos, double *dts);
double ionmodel(const GTime& t, const double *pos, const double *azel);
