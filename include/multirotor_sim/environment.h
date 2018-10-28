#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "geometry/quat.h"
#include "utils.h"
#include "nanoflann_eigen/nanoflann_eigen.h"
#include "utils.h"

using namespace Eigen;
using namespace std;
using namespace quat;
using namespace nanoflann;

class Environment
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    bool get_center_img_center_on_ground_plane(const Vector3d& t_I_c, const Quatd& q_I_c, Vector3d& point);

    int add_point(const Vector3d& t_I_c, const Quatd& q_I_c, Vector3d& zeta, Vector2d& pix, double& depth);
    bool get_closest_points(const Vector3d &query_pt, int num_pts, double max_dist,
                            vector<Vector3d, aligned_allocator<Vector3d> > &pts, vector<size_t> &ids);
    inline const std::vector<Vector3d, aligned_allocator<Vector3d>>& get_points() const {return points_.pts; }
    int point_idx_;
    
protected:
    KDTree3d* kd_tree_;
    PointCloud<double> points_;
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_;
    std::normal_distribution<double> normal_;
    double move_stdev_;
    double floor_level_;
    double max_offset_;
    Vector2d img_size_;
    Vector2d img_center_;
    Vector2d finv_;
};
