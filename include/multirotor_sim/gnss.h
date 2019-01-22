#pragma once

#include <fstream>
#include <vector>

#include <Eigen/Core>

#include "multirotor_sim/satellite.h"
#include "multirotor_sim/wsg84.h"

namespace multirotor_sim {

class GNSS
{
    void loadFromFile(std::string filename);

    std::vector<Satellite> satellites_;
};

}
