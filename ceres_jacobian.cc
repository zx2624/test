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
Eigen::Matrix3d unsym_mat(Eigen::Vector3d vec) {
  Eigen::Matrix3d mat;
  mat << 0, -1 * vec(2), vec(1), vec(2), 0, -1 * vec(0), -1 * vec(1), vec(0), 0;
  return mat;
}
void mat2RPY(const Eigen::Matrix<double, 3, 3>& m, double& roll, double& pitch,
             double& yaw) {
  roll = atan2(m(2, 1), m(2, 2));
  pitch = atan2(-m(2, 0), sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2)));
  yaw = atan2(m(1, 0), m(0, 0));
}
struct struct1 {
  int a = 2;
};
struct struct2 : public struct1 {
  int b = 3;
};
void test(std::shared_ptr<struct1> stru) { std::cout << stru->a << std::endl; }
void scout(int a) { std::cout << a << std::endl; }
void scout(double a) { std::cout << a << std::endl; }
void scout(std::string a) { std::cout << a << std::endl; }
template <typename T>
void multi_arg(T args) {
  std::cout << args << std::endl;
}
template <typename T, typename... Args>
void multi_arg(T arg, Args... args) {
  scout(arg);
  multi_arg(args...);
}

struct TestFactor {
  TestFactor() {}

  template <typename T>
  bool operator()(const T* const qs, T* residuals) const {
    // Eigen::Quaternion<T> Rc_w(pose_q[3], pose_q[0], pose_q[1], pose_q[2]);
    Eigen::Quaternion<T> q0(qs[0], qs[1], qs[2], qs[3]);
    Eigen::Matrix<T, 3, 1> v(1, 2, 3);
    v = q0 * v;
    residuals[0] = v(0);
    residuals[2] = v(1);
    residuals[2] = v(2);

    return true;
  }
  static ceres::CostFunction* Create() {
    return (
        new ceres::AutoDiffCostFunction<TestFactor, 3, 4>(new TestFactor()));
  }
};

void CRSToDenseMatrix(const ceres::CRSMatrix& input, Eigen::MatrixXd* output) {
  CHECK(output != nullptr);
  Eigen::MatrixXd& m = *output;
  m.resize(input.num_rows, input.num_cols);
  m.setZero();
  for (int row = 0; row < input.num_rows; ++row) {
    for (int j = input.rows[row]; j < input.rows[row + 1]; ++j) {
      const int col = input.cols[j];
      m(row, col) = input.values[j];
    }
  }
}

int main() {
  
  ceres::Problem prob;

}
