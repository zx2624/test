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
  std::string str = "name";
  int i = 12;
  str  = str + std::to_string(i);
  std::cout << str << std::endl;
  //0710
  double yaw = 1.5342, pitch =  0.0047, roll = 1.5214;
  //0104
  // double yaw = 1.4330470610715276, pitch =  -1.493158152010737, roll = 0.1380770137450042;
  Eigen::Matrix3d mat =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
  Eigen::Quaterniond q; q = mat;
  std::cout << "w:" << q.w() << " "
  << "x:" << q.x() << " "
  << "y:" << q.y() << " "
  << "z:" << q.z() << std::endl;
  // std::cout << mat.inverse() << std::endl
  // << mat.inverse().eval() << std::endl;
  int a  = 14;
  if(static_cast<unsigned char>(a) == HobotLabels::moving_object){
    std::cout << "================" << std::endl;
  }
  std::set<int> st;
  st.insert(1);st.insert(2);
  if(st.find(1) != st.end()){
    std::cout << "got 1111" << std::endl;

  }
  if(st.find(3) == st.end()){
    std::cout << "nnnnnnnnnnnnnn" << std::endl;
  }
}