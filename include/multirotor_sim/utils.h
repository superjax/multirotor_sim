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

template <typename Derived>
bool get_yaml_diag(const std::string key, const std::string filename, Eigen::MatrixBase<Derived>& val, bool print_error=true)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> diag;
  if (get_yaml_eigen(key, filename, diag, print_error))
  {
    val = diag.asDiagonal();
    return true;
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
      return true;
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

template<typename Derived>
Derived randomNormal(double stdev, std::normal_distribution<typename Derived::Scalar>& dist, std::default_random_engine& gen)
{
  Derived vec;
  for (int i = 0; i < Derived::RowsAtCompileTime; ++i)
    vec(i) = stdev*dist(gen);
  return vec;
}

inline bool isNan(const Eigen::Ref<const Eigen::MatrixXd>& A)
{
  return (A.array() != A.array()).any();
}

