#include <gtest/gtest.h>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/utils.h"
#include "multirotor_sim/test_common.h"

static const double RAD2DEG = 180.0 / M_PI;
static const double DEG2RAD = M_PI / 180.0;


TEST (GPS, lla2ecef)
{
    Vector3d lla = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.997511}; // BYU Campus
    Vector3d ecef_known = {-1798810.23, -4532232.54, 4099784.74};

    Vector3d ecef_calc = WSG84::lla2ecef(lla);

    ASSERT_MAT_NEAR(ecef_known, ecef_calc, 1e-2);
}

TEST (GPS, ecef2lla)
{
    Vector3d ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vector3d lla_known = {40.246184 * DEG2RAD , -111.647769 * DEG2RAD, 1387.998309};

    Vector3d lla_calc = WSG84::ecef2lla(ecef);

    ASSERT_MAT_NEAR(lla_known, lla_calc, 1e-6);
}

TEST (GPS, ecef2ned)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);
    Vector3d ned1 = {54.759, 2.481, 4.404};

    xform::Xformd x_e2n = WSG84::ecef2ned(ecef0);
    Vector3d ned1_hat = x_e2n.transformp(ecef1);

    EXPECT_MAT_NEAR(ned1_hat, ned1, 1e-3);
}

TEST (GPS, ned2ecef)
{
    Vector3d lla0 = {40.247082 * DEG2RAD, -111.647776 * DEG2RAD, 1387.998309};
    Vector3d ecef0 = WSG84::lla2ecef(lla0);

    Vector3d ned1 = {54.759, 2.481, 4.404};
    Vector3d lla1 = {40.246587 * DEG2RAD, -111.647761 * DEG2RAD, 1387.998309};
    Vector3d ecef1 = WSG84::lla2ecef(lla1);

    xform::Xformd x_e2n = WSG84::ecef2ned(ecef0);
    Vector3d ecef1_hat = x_e2n.transforma(ned1);

    EXPECT_MAT_NEAR(ecef1_hat, ecef1, 1e-3);
}

TEST (GPS, loading)
{

}
