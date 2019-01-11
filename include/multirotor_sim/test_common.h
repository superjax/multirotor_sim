#pragma once
#include <gtest/gtest.h>

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
