#pragma once
#include <stdexcept>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <random>
#include <experimental/filesystem>
#include <boost/algorithm/string.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <geometry/xform.h>

using namespace Eigen;

inline bool file_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

inline std::string current_working_dir( void ) {
  char buff[FILENAME_MAX];
  char* ptr = getcwd( buff, FILENAME_MAX );
  (void)ptr;
  std::string current_working_dir(buff);
  return current_working_dir;
}

template <typename T>
bool get_yaml_node(const std::string key, const std::string filename, T& val, bool print_error = true) 
{
  // Try to load the YAML file
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(filename);
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file " << filename << std::endl;
  }

  // Throw error if unable to load a parameter
  if (node[key])
  {
    val = node[key].as<T>();
    return true;
  }
  else
  {
    if (print_error)
    {
      throw std::runtime_error("Unable to load " + key + " from " + filename);
    }
    return false;
  }
}
template <typename Derived1>
bool get_yaml_eigen(const std::string key, const std::string filename, Eigen::MatrixBase<Derived1>& val, bool print_error=true)
{
  YAML::Node node = YAML::LoadFile(filename);
  std::vector<double> vec;
  if (node[key])
  {
    vec = node[key].as<std::vector<double>>();
    if (vec.size() == (val.rows() * val.cols()))
    {
      int k = 0;
      for (int i = 0; i < val.rows(); i++)
      {
        for (int j = 0; j < val.cols(); j++)
        {
          val(i,j) = vec[k++];
        }
      }
      return true;
    }
    else
    {
      throw std::runtime_error("Eigen Matrix Size does not match parameter size for " + key + " in " + filename +
                               ". Requested " + std::to_string(Derived1::RowsAtCompileTime) + "x" + std::to_string(Derived1::ColsAtCompileTime) +
                               ", Found " + std::to_string(vec.size()));
      return false;
    }
  }
  else if (print_error)
  {
    throw std::runtime_error("Unable to load " + key + " from " + filename);
  }
  return false;
}

template <typename T>
bool get_yaml_priority(const std::string key, const std::string file1, const std::string file2, T& val)
{
  if (get_yaml_node(key, file1, val, false))
  {
    return true;
  }
  else
  {
    return get_yaml_node(key, file2, val, true);
  }
}

template <typename Derived1>
bool get_yaml_priority_eigen(const std::string key, const std::string file1, const std::string file2, Eigen::MatrixBase<Derived1>& val)
{
  if (get_yaml_eigen(key, file1, val, false))
  {
    return true;
  }
  else
  {
    return get_yaml_eigen(key, file2, val, true);
  }
}

inline bool createDirIfNotExist(const std::string& dir)
{
  if(!std::experimental::filesystem::exists(dir))
    return std::experimental::filesystem::create_directory(dir);
  else
    return false;
}

inline std::vector<std::string> split(const std::string& s, const char* delimeter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimeter[0]))
   {
      tokens.push_back(token);
   }
   return tokens;
}

inline std::string baseName(const std::string& path)
{
  std::string filename = split(path, "/").back();
  return split(filename, ".")[0];
}


namespace frame_helper
{

// rotation from vehicle-2 to body frame
inline Eigen::Matrix3d R_v2_to_b(double phi)
{
  Eigen::Matrix3d R_v22b;
  R_v22b << 1,         0,        0,
      0,  cos(phi), sin(phi),
      0, -sin(phi), cos(phi);
  return R_v22b;
}


// rotation from vehicle-1 to vehicle-2 frame
inline Eigen::Matrix3d R_v1_to_v2(double theta)
{
  Eigen::Matrix3d R_v12v2;
  R_v12v2 << cos(theta), 0, -sin(theta),
      0, 1,           0,
      sin(theta), 0,  cos(theta);
  return R_v12v2;
}

// rotation from vehicle to vehicle-1 frame
inline Eigen::Matrix3d R_v_to_v1(double psi)
{
  Eigen::Matrix3d R_v2v1;
  R_v2v1 <<  cos(psi), sin(psi), 0,
      -sin(psi), cos(psi), 0,
      0,        0, 1;
  return R_v2v1;
}

// rotation from vehicle to body frame (3-2-1 Euler)
inline Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi)
{
  return R_v2_to_b(phi) * R_v1_to_v2(theta) * R_v_to_v1(psi);
}
}

template <typename T>
T sat(const T val, const T max, const T min)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

template <typename T>
T sat(const T max, const T val)
{
  if (val > max)
    return max;
  if (val < -1.0 * max)
    return -1.0 * max;
  return val;
}

class ProgressBar
{
public:
  ProgressBar(){}
  ProgressBar(int total, int barwidth) :
    initialized_(false),
    barwidth_(barwidth),
    total_(total)
  {}
  
  ~ProgressBar()
  {
//    std::cout << std::endl;
  }
  
  void init(int total, int barwidth)
  {
    initialized_ = false;
    barwidth_ = barwidth;
    total_ = total;
    last_completed_ = 0;
  }

  void set_theme_line() { bars_ = {"─", "─", "─", "╾", "╾", "╾", "╾", "━", "═"}; }
  void set_theme_circle() { bars_ = {" ", "◓", "◑", "◒", "◐", "◓", "◑", "◒", "#"}; }
  void set_theme_braille() { bars_ = {" ", "⡀", "⡄", "⡆", "⡇", "⡏", "⡟", "⡿", "⣿" }; }
  void set_theme_braille_spin() { bars_ = {" ", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠇", "⠿" }; }
  
  void print(int completed)
  {
    if (!initialized_)
    {
      last_print_time_ = std::chrono::system_clock::now();
      start_time_ = std::chrono::system_clock::now();
      initialized_ = true;
    }
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    // limit printing to about 30 Hz
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count() > 33
        || completed == total_)
    {
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() / 1000.0;
      last_print_time_ = now;
      std::cout << " \r [";
      double pos = barwidth_ * (completed / (double)total_);
      for (int i = 0; i < barwidth_; ++i)
        if (i < floor(pos)) std::cout << *(bars_.end()-1);
        else if (i == floor(pos)) std::cout << bars_[round((pos - floor(pos)) * (bars_.size() -1))];
        else std::cout << " ";
      std::cout << "]  ";
      printf("%.0f%% ", (completed / (double)total_)*100.0);
      double it_s = completed / elapsed;
      std::string left_stamp = ms_to_stamp(((total_ - completed) / it_s)*1000);
      std::string elapsed_stamp = ms_to_stamp(elapsed * 1000.0);
      printf("[%s<%s, %.2fit/s] ", elapsed_stamp.c_str(), left_stamp.c_str(), it_s);
      std::cout.flush();
    }
    last_completed_ = completed;
  }

  void finished()
  {
    print(total_);
  }
private:

  std::string ms_to_stamp(int ms)
  {
    if (ms <= 0.0)
    {
      return "";
    }
    int millis = ms % 1000;
    int sec = ((ms - millis) % (60 * 1000)) / 1000;
    int min = ((ms - (millis + sec*1000)) % (60 * 60 * 1000)) / (60*1000);
    int hour = ((ms - (millis + (sec + min*60)*1000)) % (24 * 60 * 60 * 1000)) / (60*60*1000);
    int day = ((ms - (millis + (sec + (min + hour * 60) * 60) * 1000)) / (24 * 60 * 60 * 1000))/(24*60*60*1000);
    char buf[25];
    int n;
    if (day > 0)
      n = sprintf(buf, "%d:%d:%02d:%02d:%03d", day, hour, min, sec, millis);
    else if (hour > 0)
      n = sprintf(buf, "%d:%02d:%02d:%03d", hour, min, sec, millis);
    else if (min > 0)
      n = sprintf(buf, "%d:%02d:%03d", min, sec, millis);
    else if (sec > 0)
      n = sprintf(buf, "%d:%03d", sec, millis);
    else
      n = sprintf(buf, "%d", millis);
    std::string out(buf);
    return out;
  }



//  }

  int barwidth_;
  int total_;
  bool initialized_;
  int last_completed_;
  std::vector<const char*> bars_ = {" ", "▏", "▎", "▍", "▋", "▋", "▊", "▉", "▉", "█"};
  
  std::chrono::system_clock::time_point start_time_;
  std::chrono::system_clock::time_point last_print_time_;  
};

class InputParser{
public:
  InputParser (int &argc, char **argv)
  {
    for (int i=1; i < argc; ++i)
      this->tokens.push_back(std::string(argv[i]));
  }

  template<typename T>
  bool getCmdOption(const std::string &option, T& ret) const
  {
    std::stringstream ss;
    auto itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end())
    {
      ss << *itr;
      ss >> ret;
    }
    return false;
  }

  bool cmdOptionExists(const std::string &option) const
  {
    return std::find(this->tokens.begin(), this->tokens.end(), option)
        != this->tokens.end();
  }

private:
  std::vector <std::string> tokens;
};

template<typename T, int S>
void random_normal_vec(Eigen::Matrix<T,S,1>& vec, double stdev, std::normal_distribution<T>& dist, std::default_random_engine& gen)
{
  for (int i = 0; i < S; ++i)
    vec(i) = stdev*dist(gen);
}

inline bool isNan(const Eigen::Ref<const Eigen::MatrixXd>& A)
{
  return (A.array() != A.array()).any();
}

struct WSG84
{
    static constexpr double A = 6378137.0;       // WGS-84 Earth semimajor axis (m)
    static constexpr double B = 6356752.314245;  // Derived Earth semiminor axis (m)
    static constexpr double F = (A - B) / A;     // Ellipsoid Flatness
    static constexpr double F_INV = 1.0 / F;     // Inverse flattening
    static constexpr double A2 = A * A;
    static constexpr double B2 = B * B;
    static constexpr double E2 = F * (2 - F);    // Square of Eccentricity

    static Vector3d ecef2lla(const Vector3d& ecef)
    {
        Vector3d lla;
        ecef2lla(ecef, lla);
        return lla;
    }

    static void ecef2lla(const Vector3d& ecef, Vector3d& lla)
    {
        static const double e2 = F * (2.0 - F);

        double r2 = ecef.x()*ecef.x() + ecef.y()*ecef.y();
        double z=ecef.z();
        double v;
        double zk;
        do
        {
            zk = z;
            double sinp = z / std::sqrt(r2 + z*z);
            v = A / std::sqrt(1.0 - e2*sinp*sinp);
            z = ecef.z() + v*e2*sinp;
        }
        while (std::abs(z - zk) >= 1e-4);

        lla.x() = r2 > 1e-12 ? std::atan(z / std::sqrt(r2)) : (ecef.z() > 0.0 ? M_PI/2.0 : -M_PI/2.0);
        lla.y() = r2 > 1e-12 ? std::atan2(ecef.y(), ecef.x()) : 0.0;
        lla.z() = std::sqrt(r2+z*z) - v;
    }

    static Vector3d lla2ecef(const Vector3d& lla)
    {
        Vector3d ecef;
        lla2ecef(lla, ecef);
        return ecef;
    }

    static void lla2ecef(const Vector3d& lla, Vector3d& ecef)
    {
        double sinp=sin(lla[0]);
        double cosp=cos(lla[0]);
        double sinl=sin(lla[1]);
        double cosl=cos(lla[1]);
        double e2=F*(2.0-F);
        double v=A/sqrt(1.0-e2*sinp*sinp);

        ecef[0]=(v+lla[2])*cosp*cosl;
        ecef[1]=(v+lla[2])*cosp*sinl;
        ecef[2]=(v*(1.0-e2)+lla[2])*sinp;
    }

    static void x_ecef2ned(const Vector3d& ecef, xform::Xformd& X_e2n)
    {
        X_e2n.q() = q_e2n(ecef2lla(ecef));
        X_e2n.t() = ecef;
    }

    static xform::Xformd x_ecef2ned(const Vector3d& ecef)
    {
        xform::Xformd X_e2n;
        x_ecef2ned(ecef, X_e2n);
        return X_e2n;
    }

    static Vector3d ned2ecef(const xform::Xformd x_e2n, const Vector3d& ned)
    {
        return x_e2n.transforma(ned);
    }

    static void ned2ecef(const xform::Xformd x_e2n, const Vector3d& ned, Vector3d& ecef)
    {
        ecef = x_e2n.transforma(ned);
    }

    static Vector3d ecef2ned(const xform::Xformd x_e2n, const Vector3d& ecef)
    {
        return x_e2n.transformp(ecef);
    }

    static void ecef2ned(const xform::Xformd x_e2n, const Vector3d& ecef, Vector3d& ned)
    {
        ned = x_e2n.transformp(ecef);
    }

    static void lla2ned(const Vector3d& lla0, const Vector3d& lla, Vector3d& ned)
    {
        xform::Xformd x_e2n;
        x_e2n.q() = q_e2n(lla0);
        x_e2n.t() = lla2ecef(lla0);
        ecef2ned(x_e2n, lla2ecef(lla), ned);
    }

    static Vector3d lla2ned(const Vector3d& lla0, const Vector3d& lla)
    {
        Vector3d ned;
        lla2ned(lla0, lla, ned);
        return ned;
    }

    static void ned2lla(const Vector3d& lla0, const Vector3d& ned, Vector3d&lla)
    {
        xform::Xformd x_e2n;
        x_e2n.q() = q_e2n(lla0);
        x_e2n.t() = lla2ecef(lla0);
        ecef2lla(ned2ecef(x_e2n, ned), lla);
    }

    static Vector3d ned2lla(const Vector3d& lla0, const Vector3d& ned)
    {
        Vector3d lla;
        ned2lla(lla0, ned, lla);
        return lla;
    }

    static quat::Quatd q_e2n(const Vector3d& lla)
    {
        Quatd q1, q2;
        q1 = quat::Quatd::from_axis_angle(e_z, lla(1));
        q2 = quat::Quatd::from_axis_angle(e_y, -M_PI/2.0 - lla(0));
        return q1 * q2;
    }
};


