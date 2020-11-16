#include <ceres/ceres.h>
#include <pcl/common/transforms.h>
#include <stdarg.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <vector>

#include "factor/factor.h"
#include "testlib.h"
#include "tools/tools.h"
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
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
class B;
class A {
  friend class B;

 public:
  A() { std::cout << "empty A" << std::endl; }

 private:
  void add() { std::cout << "this is a add in A" << std::endl; }
};
class B {
 public:
  B() { std::cout << "empty bB" << std::endl; }

 public:
  void A_add() {
    std::cout << "got add from A" << std::endl;
    a.add();
  }
  A a;
};

int main() {
  PointXYZIRPYT pt;
  pt.roll = 13;
  std::cout << pt.roll << std::endl;
}
