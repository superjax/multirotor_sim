#pragma once

#include <Eigen/Core>

#include "quat.h"
#include "utils.h"

using namespace Eigen;
using namespace std;
using namespace quat;

class Environment
{
    typedef struct
    {
        Vector3d bottom_left;
        Vector3d normal;
        double width;
        double height;
    } wall_t;
    
public:
    Environment(int seed);
    void load(std::string filename);

    int add_point(const Vector3d& t_I_c, const Quatd& q_I_c, Vector3d& zeta, Vector2d& pix, double& depth);
    void move_point(int id);
    const Matrix<double, Dynamic, 3>& get_points() const;
    int point_idx_;
    
protected:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_;
    std::normal_distribution<double> normal_;
    double move_stdev_;
    double floor_level_;
    Matrix<double, Dynamic, 3> points_;
    double max_offset_;
    Vector2d img_size_;
    Vector2d img_center_;
    Vector2d finv_;
};
