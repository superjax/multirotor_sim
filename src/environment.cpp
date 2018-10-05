#include "environment.h"
#include "support.h"
    
Environment::Environment(int seed)
  : uniform_(-1.0, 1.0),
    generator_(seed)
{}

void Environment::load(string filename)
{
  get_yaml_node("wall_max_offset", filename, max_offset_);
  get_yaml_node("points_move_stdev", filename, move_stdev_);
  get_yaml_eigen("image_size", filename, img_size_);
  Vector2d focal_len;
  get_yaml_eigen("focal_len", filename, focal_len);
  get_yaml_eigen("cam_center", filename, img_center_);
  finv_ = focal_len.cwiseInverse();


  point_idx_ = 0;
  points_.resize(4096,3);
  points_.setZero();
  floor_level_ = 0;
}

int Environment::add_point(const Vector3d& t_I_c, const Quatd& q_I_c, Vector3d& zeta, Vector2d& pix, double& depth)
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
    points_.row(idx) = new_point;
    if (point_idx_ >= points_.rows())
    {
      points_.conservativeResize(points_.rows() + 4096,3);
    }
    return idx;
  }
}

void Environment::move_point(int id)
{
  Vector3d move;
  random_normal_vec(move, move_stdev_, normal_, generator_);
//  cout << "moving point by " << move.transpose();
  points_.row(id) += move.transpose();
}

const Matrix<double, Dynamic, 3>& Environment::get_points() const
{
    return points_;
}
