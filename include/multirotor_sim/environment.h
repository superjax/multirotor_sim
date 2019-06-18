#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "geometry/xform.h"
#include "nanoflann_eigen/nanoflann_eigen.h"
#include "multirotor_sim/utils.h"
#include "multirotor_sim/state.h"

namespace multirotor_sim
{

class Environment
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef struct
    {
        Eigen::Vector3d bottom_left;
        Eigen::Vector3d normal;
        double width;
        double height;
    } wall_t;

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3;
    
public:
    Environment(int seed);
    void load(std::string filename);
    bool get_center_img_center_on_ground_plane(const xform::Xformd &x_I2c, Eigen::Vector3d& point);

    int add_point(const Eigen::Vector3d& t_I_c, const quat::Quatd& q_I_c,
                  const FeatVec &tracked, Eigen::Vector3d& zeta, Eigen::Vector2d& pix, double& depth);
    bool get_closest_points(const Eigen::Vector3d &query_pt, int num_pts, double max_dist,
                            VecVec3 &pts, std::vector<size_t> &ids);
    inline const VecVec3& get_points() const {return points_.pts; }
    int point_idx_;
    
protected:
    KDTree3d* kd_tree_;
    PointCloud<double> points_;
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_;
    std::normal_distribution<double> normal_;
    double floor_level_;
    double max_offset_;
    Eigen::Vector2d img_size_;
    Eigen::Vector2d img_center_;
    Eigen::Vector2d finv_;
    double feat_too_close_thresh_;
};

}
