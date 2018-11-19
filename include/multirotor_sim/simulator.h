#pragma once

#include <random>
#include <algorithm>
#include <deque>
#include <fstream>

#include "Eigen/Core"

#include "utils.h"
#include "environment.h"
#include "dynamics.h"
#include "controller.h"

#include "geometry/quat.h"

#define NUM_FEATURES 12
#define MAX_X 17+NUM_FEATURES*5
#define MAX_DX 16+NUM_FEATURES*3

//#ifndef NDEBUG
//#define DBG(msg, ...)\
//  { \
//    printf((msg), ##__VA_ARGS__); \
//  }
//#else
#define DBG(msg, ...) {}
//#endif

using namespace quat;

class Simulator
{
public:
  typedef enum {
      ACC,
      ALT,
      ATT,
      POS,
      VEL,
      QZETA,
      FEAT,
      PIXEL_VEL,
      DEPTH,
      INV_DEPTH,
      VO,
      TOTAL_MEAS
    } measurement_type_t;

  enum : int{
      xPOS = 0,
      xVEL = 3,
      xATT = 6,
      xB_A = 10,
      xB_G = 13,
      xMU = 16,
      xZ = 17
    };

  typedef Matrix<double, MAX_X, 1> xVector;
  typedef Matrix<double, MAX_DX, 1> dxVector;

  typedef struct
  {
    double t;
    measurement_type_t type;
    VectorXd z;
    MatrixXd R;
    int feature_id;
    double depth;
    bool active;
  } measurement_t;
  
  Simulator();
  ~Simulator();
  Simulator(bool prog_indicator);
  Simulator(bool prog_indicator, uint64_t seed);
  
  /**
   * @brief load
   * loads parameters from file
   * @param filename
   */
  void load(std::string filename);
  
  /**
   * @brief run
   * Main driver of the simulation loop
   * @return true if not yet at final time, false if complete
   */
  bool run();
  
  /**
   * @brief get_measurements
   * returns a list of measurements which occurred since the last get_measurements call
   * these measurements all occur at the current time
   * @param meas
   */
  void get_measurements(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);
  void get_imu_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);
  void get_feature_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);
  void get_alt_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);
  void get_mocap_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);

  /**
   * @brief get_vo_meas
   * Computes the relative pose from current camera to keyframe camera and
   * adds it to the measurement list.
   * Relative position is given in current camera frame pointing from current
   * camera to keyframe camera.
   * Relative rotation is the rotation from current camera frame to keyframe
   * camera frame.
   * @param meas
   */
  void get_vo_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);


  const Vector6d& get_imu_prev() const { return imu_prev_; }

  /**
   * @brief get_pose
   * Returns the current (true) state of the multirotor
   * @returns current pose (position, attitude)
   */
  Xformd get_pose() const;

  /**
   * @brief get_truth
   * @return the full truth vector (as supplied by EKF class)
   */
  void get_truth(xVector& x, const std::vector<int> &tracked_features) const;

  /**
   * @brief get_imu_noise
   * @return 6x6 diagonal matrix of IMU covariance [acc, gyro]
   */
  Matrix6d get_imu_noise_covariance() const;

  /**
   * @brief get_imu_noise
   * @return 6x6 diagonal matrix of motion capture measurement [t, q]
   */
  Matrix6d get_mocap_noise_covariance() const;

  inline Vector3d get_accel_bias() const { return accel_bias_; }
  inline Vector3d get_gyro_bias() const { return gyro_bias_; }

  void log_state();
  
  Environment env_;
  dynamics::Dynamics dyn_;
  controller::Controller cont_;
  double t_, dt_, tmax_;

  Matrix3d att_R_;
  Matrix3d pos_R_;
  Matrix3d acc_R_;
  Matrix2d feat_R_;
  Matrix<double, 1, 1> alt_R_;
  Matrix<double, 1, 1> depth_R_;
  Matrix<double, 6, 6> vo_R_;
  
  bool attitude_update_active_;
  bool position_update_active_;
  bool depth_update_active_;
  bool feature_update_active_;
  bool drag_update_active_;
  bool altimeter_update_active_;
  bool vo_update_active_;


  typedef struct
  {
    Vector3d zeta;
    Vector2d pixel;
    double depth;
    int id;
  } feature_t;
  
  /**
   * @brief update_feature
   * This function updates the zeta, pixel and depth fields
   * of the supplied feature object given the current
   * simulator state
   * @param feature
   * @return true if feature still in image, false otherwise
   */
  bool update_feature(feature_t &feature) const;
  
  /**
   * @brief proj
   * Calculates the Projection of bearing vector zeta with
   * Simulator class Camera Intrinsic parameters
   * @param zeta - bearing vector (normalized)
   * @param pix - pixel location (in image frame)
   */
  void proj(const Vector3d& zeta, Vector2d& pix) const;
  
  /**
   * @brief get_feature_in_frame
   * If retrack is true first tries to find pixels previously tracked in the frame that are not
   * currently being tracked.  If retrack is false, or there are no previously observed landmarks,
   * or all previously observed landmarks are being tracked, creates a new, randomly selected landmark
   * in the camera frame from the environment and calculates its projection - global_id is automatically
   * incremented with each new landmark
   * @param feature - ouput new feature with projection loaded
   * @param retrack - flag of whether to rediscover old landmarks or always create new
   * @returns bool - true if suceeded, false if the ground plane is not in the camera FOV.
   */
  bool get_feature_in_frame(feature_t &feature, bool retrack);


  /**
   * @brief get_previously_tracked_feature_in_frame
   * @param feature
   * @return true if success
   */
  bool get_previously_tracked_feature_in_frame(feature_t &feature);

  /**
   * @brief create_new_feature_in_frame
   * @param feature
   * @return true if success
   */
  bool create_new_feature_in_frame(feature_t &feature);

  
  /**
   * @brief is_feature_tracked
   * Returns true if the landmark referred to by env_id is already in the tracked_features_ list
   * @param env_id - the id of the point in the environment points array
   * @return true if tracked, false otherwise
   */
  bool is_feature_tracked(const int env_id) const;
  
  /**
   * @brief update_camera_pose
   * Updates the q_I_c_ and t_I_c_ objects (the camera pose wrt to the inertial frame).
   * given the current simulator state.  This needs to be called before update_feature()
   * after a call to run()
   */
  void update_camera_pose();
  
  
  /**
   * @brief get_attitude
   * @return current attitude, sampled from normal distribution centered at true mean
   * with covariance R_att_
   */
  Vector4d get_attitude();
  
  /**
   * @brief get_position
   * @return current position, sampled from normal distribution centered at true mean
   * with covariance R_pos_
   */
  Vector3d get_position();
  
  /**
   * @brief get_pixel
   * @param feature
   * @return current pixel measurement of feature sampled from normal distribution 
   * centered at true mean with covariance R_feat_
   */
  Vector2d get_pixel(const feature_t& feature);
  
  /**
   * @brief get_depth
   * @param feature
   * @return current feature depth of feature sampled from normal distribution 
   * centered at true mean with variance R_depth_
   */
  double get_depth(const feature_t& feature, bool override= false);
  
  /**
   * @brief get_altitude
   * @return current altitude, sampled from normal distribution centered at true mean
   * with variance R_alt_
   */
  Matrix<double, 1, 1> get_altitude();
  
  /**
   * @brief get_acc
   * @return current acceleration, sampled from normal distribution centered at true mean
   * with covariance R_acc_
   */
  Vector3d get_acc();

  // Progress indicator, updated by run()
  ProgressBar prog_;
  bool prog_indicator_;
  std::string log_filename_;
  ofstream log_;

  // Command vector passed from controller to dynamics
  dynamics::commandVector u_;
  
  /* === Sensors === */
  uint64_t seed_;
  default_random_engine generator_;
  uniform_real_distribution<double> uniform_;
  normal_distribution<double> normal_;

  // IMU
  Vector6d imu_; // Vector containing [accel;gyro]
  Vector6d imu_prev_; // Previous version for propagation
  double imu_update_rate_; // determines how often to generate an altitude measurement
  double last_imu_update_;
  Quatd q_b_u_;
  Vector3d p_b_u_;

  bool use_accel_truth_;
  Vector3d accel_bias_; // Memory for random walk
  double accel_noise_stdev_; // Standard deviation of accelerometer noise
  double accel_walk_stdev_; // Strength of accelerometer random walk
  Vector3d accel_noise_;

  bool use_gyro_truth_;
  Vector3d gyro_bias_; // Memory for random walk
  double gyro_noise_stdev_; // Standard deviation of gyro noise
  double gyro_walk_stdev_; // Strength of gyro random walk
  Vector3d gyro_noise_;
  
  // Camera
  bool use_camera_truth_;
  double pixel_noise_stdev_; // Standard deviation of camera pixel noise
  vector<Vector3d> cam_; // Elements are landmarks in camera FOV given by a vector containing [pixel_x,pixel_y,label]
  double camera_update_rate_;  // determines how often to supply a set of feature measurements
  double last_camera_update_;
  int next_feature_id_;
  vector<feature_t, aligned_allocator<feature_t>> tracked_points_; // currently tracked features
  deque<measurement_t, aligned_allocator<measurement_t>> camera_measurements_buffer_; // container to hold measurements while waiting for delay
  double camera_time_delay_;
  Vector2d pixel_noise_;
  bool loop_closure_;
  
  // Pose of camera in the body frame - assumed to be fixed
  Quatd q_b_c_;
  Vector3d p_b_c_;
  
  // Pose of camera in the inertial frame, updated by update_camera_pose()
  Quatd q_I_c_;
  Vector3d t_I_c_;
  double feat_move_prob_;
  
  // Camera intrinsics - assmed to be fixed
  Matrix<double, 2, 3> cam_F_;
  Vector2d cam_center_;
  Vector2d image_size_;

  // Altimeter
  bool use_altimeter_truth_;
  double altimeter_update_rate_; // determines how often to generate an altitude measurement
  double last_altimeter_update_;
  double altimeter_noise_stdev_; // Standard deviation of altimeter noise
  double altimeter_noise_;

  // Depth
  bool use_depth_truth_;
  bool init_depth_;
  double depth_update_rate_; // determines how often to generate an altitude measurement
  double last_depth_update_;
  double depth_noise_stdev_; // Standard deviation of altimeter noise
  double depth_noise_;

  // Visual Odometry
  xform::Xformd T_i2bk_; // Inertial to body keyframe pose
  bool use_vo_truth_;
  double vo_delta_position_;
  double vo_delta_attitude_;
  double vo_translation_noise_stdev_;
  double vo_rotation_noise_stdev_;

  // Truth
  Quatd q_b_m_;
  Vector3d p_b_m_;
  bool use_attitude_truth_;
  bool use_position_truth_;
  double truth_update_rate_; // determines how often to supply truth measurements
  double attitude_noise_stdev_;
  double position_noise_stdev_;
  double last_truth_update_ = 0.0;
  double next_truth_measurement_ = 0.0;
  double mocap_time_offset_;
  double mocap_transmission_noise_;
  double mocap_transmission_time_;
  deque<std::pair<double, measurement_t>, aligned_allocator<std::pair<double, measurement_t>>> mocap_measurement_buffer_; // container to hold measurements while waiting for delay

};
