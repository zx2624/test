#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <set>
#include <chrono>
#include <iomanip>
#include <boost/algorithm/string.hpp>
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
int main() {

  //0710
  double yaw = 3.1415926/2 * 1.3, pitch = 0.2, roll = 0;
  //0104
  // double yaw = 1.4330470610715276, pitch =  -1.493158152010737, roll = 0.1380770137450042;
  Eigen::Matrix3d mat =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
    
      				Eigen::Matrix3d delta_mat; delta_mat << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  std::cout << mat*delta_mat << "   mat " << std::endl;


  Eigen::Quaterniond q_mat(mat);
  Eigen::Quaterniond delta_q(delta_mat);
  std::cout << (q_mat*delta_q).matrix() << std::endl;
  // q_mat.toRotationMatrix()
  Eigen::Quaterniond q; q = mat;

  q.w() = 0;
  q.z() = 0;
  q.y() = 0;
  q.x() = 0;
  
  std::cout << "w:" << q.w() << " "
  << "x:" << q.x() << " "
  << "y:" << q.y() << " "
  << "z:" << q.z() << std::endl;
  // std::cout << mat.inverse() << std::endl
  // << mat.inverse().eval() << std::endl;

  string command = "mkdir -p bddd";
  system(command.c_str());
  std::string str = "a/b/cde.bag";
  std::vector<std::string> str_vec;
  boost::split(str_vec, str, boost::is_any_of("/."));
  for(auto s : str_vec){
    std::cout << s << std::endl;
  }
    // double t = (double)(ms.count() / 1000.0f);
    // std::cout << t << std::endl;
}