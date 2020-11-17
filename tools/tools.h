#ifndef TOOLS_H_
#define TOOLS_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace tools {
// 1R, 2P, 3Y
Eigen::Matrix3d RPY2mat(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}
void mat2RPY(const Eigen::Matrix<double, 3, 3>& m, double& roll, double& pitch,
             double& yaw) {
  roll = atan2(m(2, 1), m(2, 2));
  pitch = atan2(-m(2, 0), sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2)));
  yaw = atan2(m(1, 0), m(0, 0));
}
Eigen::Vector3d mat2RPY(const Eigen::Matrix<double, 3, 3>& m) {
  double roll = atan2(m(2, 1), m(2, 2));
  double pitch = atan2(-m(2, 0), sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2)));
  double yaw = atan2(m(1, 0), m(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}
// 1Y, 2P, 3R
Eigen::Matrix3d YPR2mat(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
      .toRotationMatrix();
}
void mat2YPR(const Eigen::Matrix<double, 3, 3>& m, double& roll, double& pitch,
             double& yaw) {
  roll = atan2(-m(1, 2), m(2, 2));
  pitch = atan2(m(0, 2), sqrt(m(1, 2) * m(1, 2) + m(2, 2) * m(2, 2)));
  yaw = atan2(-m(0, 1), m(0, 0));
}

Eigen::Matrix4d GetT(const Eigen::Quaterniond& q, const Eigen::Vector3d& t){
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T.block<3, 1>(0, 3) = t;
  return T;
}

void GetRPYXYZ(const Eigen::Matrix4d& T, Eigen::Vector3d& rpy, Eigen::Vector3d& xyz){
  rpy = mat2RPY(T.block<3, 3>(0, 0));
  xyz = T.block<3, 1>(0, 3);
}
// void CRSToDenseMatrix(const ceres::CRSMatrix& input, Eigen::MatrixXd* output) {
//   CHECK(output != nullptr);
//   Eigen::MatrixXd& m = *output;
//   m.resize(input.num_rows, input.num_cols);
//   m.setZero();
//   for (int row = 0; row < input.num_rows; ++row) {
//     for (int j = input.rows[row]; j < input.rows[row + 1]; ++j) {
//       const int col = input.cols[j];
//       m(row, col) = input.values[j];
//     }
//   }
// }

}  // namespace tools
#endif  // TOOLS_H_