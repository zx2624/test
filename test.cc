#include <tf/tf.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <set>
#include <vector>
using namespace std;
// #include
enum HobotLabels {
  road = 0,
  lane = 1,
  stop_lane = 2,
  crosswalk_line = 3,
  traffic_arrow = 4,
  lane_marking = 5,
  guide_line = 6,
  speed_bump = 7,
  traffic_sign = 8,
  guide_post = 9,
  traffic_light = 10,
  pole = 11,
  building = 12,
  sidewalk = 13,
  moving_object = 14,
  background = 15
};
Eigen::Matrix3d unsym_mat(Eigen::Vector3d vec) {
  Eigen::Matrix3d mat;
  mat << 0, -1 * vec(2), vec(1), vec(2), 0, -1 * vec(0), -1 * vec(1), vec(0), 0;
  return mat;
}
int main() {
  // 0710
  double yaw = 1.586142764025011, pitch = 3.119029983253778,
         roll = -3.129042033180273;
  // 0104
  // double yaw = 1.4330470610715276, pitch =  -1.493158152010737, roll =
  // 0.1380770137450042; Eigen::Matrix3d mat =
  //        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())*
  //       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
  //       Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
  //     .matrix();
  Eigen::Matrix3d mat =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
  std::cout << mat << std::endl;
  std::cout << "======================" << std::endl;
  Eigen::Matrix3d mat_2 =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).matrix();
  std::cout << mat_2 << std::endl;

  tf::Matrix3x3 mat_tf;
  mat_tf.setRPY(roll, pitch, yaw);
  tf::Vector3 rpy_tf;
  mat_tf.getRPY(rpy_tf[0], rpy_tf[1], rpy_tf[2]);
  std::cout << rpy_tf[0] << " " << rpy_tf[1] << " " << rpy_tf[2] << " "
            << std::endl;
            
  //////////////////////
  Eigen::Quaterniond q_cam_chasis;
  q_cam_chasis.w() = 0.5277614059740843;
  q_cam_chasis.x() = 0.4506400033782436;
  q_cam_chasis.y() = -0.4934785109517533;
  q_cam_chasis.z() = 0.5242808836381074;
  Eigen::Vector3d t_cam_chsis;
  t_cam_chsis.x() = -0.1871581245824825;
  t_cam_chsis.y() = 1.250596260979218;
  t_cam_chsis.z() = -2.936984079567986;

  Eigen::Quaterniond q_chasis_lidar;
  q_chasis_lidar.x() = 0.0;
  q_chasis_lidar.y() = 0.0;
  q_chasis_lidar.z() = -0.00436330928475;
  q_chasis_lidar.w() = 0.999990480721;
  Eigen::Vector3d t_chasis_lidar;
  t_chasis_lidar.x() = 0.755;
  t_chasis_lidar.y() = 0.04;
  t_chasis_lidar.z() = 1.97;

  std::cout << "-----------now-----------" << std::endl;


  std::cout << (q_cam_chasis * q_chasis_lidar).coeffs() << std::endl;
  std::cout << "-----------now-----------" << std::endl;
  std::cout << t_cam_chsis + q_cam_chasis * t_chasis_lidar << std::endl;
  std::cout << "=============================" << std::endl;

  Eigen::Quaterniond q_chasis_gnss;
  q_chasis_gnss.x() = -0.0;
  q_chasis_gnss.y() = 0.0;
  q_chasis_gnss.z() = 0.707106781187;
  q_chasis_gnss.w() = -0.707106781187;
  Eigen::Vector3d t_chasis_gnss;
  t_chasis_gnss.x() = 1.291;
  t_chasis_gnss.y() = 0.46;
  t_chasis_gnss.z() = 1.548;

  Eigen::Quaterniond q_gnss_chasis = q_chasis_gnss.inverse();
  Eigen::Vector3d t_gnss_chasis = -(q_gnss_chasis * t_chasis_gnss);

  std::cout << (q_gnss_chasis * q_chasis_lidar).toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;

  std::cout << "-----------now-----------" << std::endl;
  std::cout << t_gnss_chasis + q_gnss_chasis * t_chasis_lidar << std::endl;
  

  std::cout << "===============" << std::endl;
  Eigen::Quaterniond qlc;
  qlc.w() = 0.5277614059740843;
  qlc.x() = 0.4506400033782436;
  qlc.y() = -0.4934785109517533;
  qlc.z() = 0.5242808836381074;

  Eigen::Vector3d tlc;
  tlc.x() = -0.1871581245824825;
  tlc.y() = 1.250596260979218;
  tlc.z() = -2.936984079567986;

  std::cout << -(qlc.inverse() * tlc) << std::endl;
}