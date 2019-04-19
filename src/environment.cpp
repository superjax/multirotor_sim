#include "geometry/support.h"
#include "nanoflann_eigen/nanoflann_eigen.h"

#include "multirotor_sim/environment.h"

    
Environment::Environment(int seed)
  : uniform_(-1.0, 1.0),
    generator_(seed)
{}

void Environment::load(string filename)
{
  get_yaml_node("wall_max_offset", filename, max_offset_);
  get_yaml_eigen("image_size", filename, img_size_);
  Vector2d focal_len;
  get_yaml_eigen("focal_len", filename, focal_len);
  get_yaml_eigen("cam_center", filename, img_center_);
  finv_ = focal_len.cwiseInverse();

  int seed;
  get_yaml_node("seed", filename, seed);
  if (seed == 0)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator_ = std::default_random_engine(seed);
  srand(seed);

  point_idx_ = 0;
  floor_level_ = 0;
  kd_tree_ = new KDTree3d(3, points_, 10);
}

bool Environment::get_center_img_center_on_ground_plane(const Xformd& x_I2c, Vector3d& point)
{
  Vector3d zeta_I = x_I2c.rota(e_z);
  double depth = -x_I2c.t()(2) / zeta_I(2);
  if (depth < 0.5 || depth > 100.0 )
  {
    return false;
  }
  else
  {
    point =  zeta_I * depth + x_I2c.t();
    return true;
  }
}

int Environment::add_point(const Vector3d& t_I_c, const quat::Quatd& q_I_c, Vector3d& zeta, Vector2d& pix, double& depth)
{
  // Choose a random pixel (assume that image center is at center of camera)
  pix.setRandom();
  pix = 0.45 * pix.cwiseProduct(img_size_); // stay away from the edges of image

  // Calculate the Unit Vector
  zeta << pix.cwiseProduct(finv_), 1.0;
  zeta /= zeta.norm();

  pix += img_center_;

  // Rotate the Unit vector into inertial coordatines
  Vector3d zeta_I = q_I_c.rota(zeta);

  // Find where zeta crosses the floor
  double h = -1.0 * (t_I_c(2) + uniform_(generator_) * max_offset_);
  depth = h / (zeta_I(2));
  if (depth < 0.5 || depth > 100.0 )
  {
    return -1;
  }
  else
  {
    int idx = point_idx_++;
    Vector3d new_point = t_I_c + depth * zeta_I;
    points_.pts.push_back(new_point);
    kd_tree_->addPoints(idx, idx);
    return idx;
  }
}

bool Environment::get_closest_points(const Vector3d& query_pt,
      int num_pts, double max_dist, vector<Vector3d, aligned_allocator<Vector3d>>& pts, vector<size_t>& ids)
{
  std::vector<size_t> ret_index(num_pts);
  std::vector<double> dist_sqr(num_pts);
  nanoflann::KNNResultSet<double> resultSet(num_pts);
  resultSet.init(&ret_index[0], &dist_sqr[0]);
  kd_tree_->findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParams(10));

  pts.clear();
  ids.clear();
  for (int i = 0; i < resultSet.size(); i++)
  {
    if (dist_sqr[i] < max_dist*max_dist)
    {
      pts.push_back(points_.pts[ret_index[i]]);
      ids.push_back(ret_index[i]);
    }
  }
  return pts.size() > 0;
}

//void Environment::move_point(int id)
//{
//  Vector3d move;
//  random_normal_vec(move, move_stdev_, normal_, generator_);
////  cout << "moving point by " << move.transpose();
//  points_.row(id) += move.transpose();
//}
