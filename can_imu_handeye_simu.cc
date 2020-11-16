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

// struct TestFactor {
//   TestFactor() {}

//   template <typename T>
//   bool operator()(const T* const qs, T* residuals) const {
//     // Eigen::Quaternion<T> Rc_w(pose_q[3], pose_q[0], pose_q[1], pose_q[2]);
//     Eigen::Quaternion<T> q0(qs[0], qs[1], qs[2], qs[3]);
//     Eigen::Matrix<T, 3, 1> v(1, 2, 3);
//     v = q0 * v;
//     residuals[0] = v(0);
//     residuals[2] = v(1);
//     residuals[2] = v(2);

//     return true;
//   }
//   static ceres::CostFunction* Create() {
//     return (
//         new ceres::AutoDiffCostFunction<TestFactor, 3, 4>(new TestFactor()));
//   }
// };

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
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> scene_change{0., 0.3};  // rad/s 转弯
  std::normal_distribution<double> scene_change_small{0., 0.1};
  std::normal_distribution<double> angle_obs{0., 0.005};

  std::normal_distribution<double> dis_obs{0., 0.5};
  std::normal_distribution<double> velocity{0, 10};
  std::normal_distribution<double> velocity_small{0, 0};

  for (int i = 0; i < 25; ++i) {
    Eigen::Vector3d gyr(0.0, 0.00, 0);
    Eigen::Vector3d vel(15, 0, 0);

    Eigen::Vector3d tic(1, 1, 1);
    Eigen::Quaterniond qci(1, 0, 0, 0);
    { qci = tools::RPY2mat(0.0, 0.0, 0.); }
    Eigen::Quaterniond qic = qci.inverse();
    Eigen::Matrix4d Tic = tools::GetT(qic, tic);
    std::normal_distribution<double> init_r_noise{0, 0.1};
    std::normal_distribution<double> init_p_noise{0, 0.1};
    std::normal_distribution<double> init_y_noise{0, 0.2};
    std::normal_distribution<double> init_xyz_noise{0, 1};
    Eigen::Quaterniond qic_op(
        tools::RPY2mat(0.0 + init_r_noise(gen), 0.0 + init_p_noise(gen),
                       0. + init_y_noise(gen)));  //(1, 0, 0, 0);
    Eigen::Vector3d tic_op(init_xyz_noise(gen), init_xyz_noise(gen),
                           init_xyz_noise(gen));  //(0, 0, 0);

    ceres::Problem prob;
    ceres::LossFunction* loss = new ceres::CauchyLoss(2.0);
    double frequency = 0.1;
    int scene_size = 10000;
    for (int i = 0; i < scene_size; ++i) {
      Eigen::Vector3d gyro_now(scene_change_small(gen), scene_change_small(gen),
                               scene_change_small(gen));
      Eigen::Vector3d vel_now =
          vel + Eigen::Vector3d(velocity(gen), velocity_small(gen),
                                velocity_small(gen));
      if (i < scene_size) {  //
        gyro_now = Eigen::Vector3d(scene_change_small(gen),
                                   scene_change_small(gen), scene_change(gen));
        vel_now = vel + Eigen::Vector3d(velocity(gen), velocity_small(gen),
                                        velocity_small(gen));
      }

      Eigen::Vector3d drpy = gyro_now * frequency;
      Eigen::Vector3d dxyz = vel_now * frequency;
      // std::cout << drpy(0) << " " << drpy(1) << " " << drpy(2) << " " <<
      // dxyz(0)
      //           << " " << dxyz(1) << " " << dxyz(2) << std::endl;

      // can truth
      Eigen::Matrix4d Tc0 = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d Tc1 = Tc0;
      Tc1.block<3, 3>(0, 0) = tools::RPY2mat(drpy(0), drpy(1), drpy(2));
      Tc1.block<3, 1>(0, 3) = dxyz;
      // imu truth
      Eigen::Matrix4d Ti0 = Tc0 * (Tic.inverse());
      Eigen::Matrix4d Ti1 = Tc1 * (Tic.inverse());

      // can measure
      Eigen::Vector3d t_c_12(dxyz(0) + dis_obs(gen), 0, 0); 
      Eigen::Quaterniond q_c_12(tools::RPY2mat(0, 0, drpy(2) + angle_obs(gen)));
      Eigen::Matrix4d T_i_12 = Ti0.inverse() * Ti1;
      Eigen::Quaterniond q_i_12(T_i_12.block<3, 3>(0, 0));
      {
        double r, p, y;
        tools::mat2RPY(T_i_12.block<3, 3>(0, 0), r, p, y);
        q_i_12 = Eigen::Quaterniond(tools::RPY2mat(
            r + angle_obs(gen), p + angle_obs(gen), y + angle_obs(gen)));
      }
      Eigen::Vector3d t_i_12(T_i_12.block<3, 1>(0, 3));
      t_i_12 += Eigen::Vector3d(dis_obs(gen), dis_obs(gen), dis_obs(gen));
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<HandEye, 6, 4, 3>(
              new HandEye(q_i_12, t_i_12, q_c_12, t_c_12));
      auto id = prob.AddResidualBlock(cost, loss, qic_op.coeffs().data(),
                                      tic_op.data());
      double costs[6];
      ceres::CRSMatrix jacobian_ceres;
      ceres::Problem::EvaluateOptions ev_option;
      ev_option.parameter_blocks =
          std::vector<double*>{qic_op.coeffs().data(), tic_op.data()};
      ev_option.residual_blocks = std::vector<ceres::ResidualBlockId>{id};
      if (i == 0) {
        ceres::LocalParameterization* local_p =
            new ceres::EigenQuaternionParameterization;
        prob.SetParameterization(qic_op.coeffs().data(), local_p);
      }
      // prob.Evaluate(ev_option, costs, NULL, NULL, &jacobian_ceres);
      // Eigen::MatrixXd jaco;
      // CRSToDenseMatrix(jacobian_ceres, &jaco);
      // Eigen::Map<Eigen::Matrix<double, 6, 1>> error(costs);
      // std::cout << "jacobian \n" << jaco << std::endl;
      // Eigen::MatrixXd ata = jaco.transpose() * jaco;
      // Eigen::MatrixXd atb = jaco.transpose() * error;
      // std::cout << "delta \n"  << ata.inverse() * atb << std::endl;
    }

    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    // options.preconditioner_type = ceres::SCHUR_JACOBI;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.min_linear_solver_iterations = 0;
    options.max_linear_solver_iterations = 500;
    options.max_num_iterations = 200;
    // options.minimizer_progress_to_stdout = true;
    options.num_threads = 48;
    options.update_state_every_iteration = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &prob, &summary);

    double r, p, y;
    tools::mat2RPY((qic_op).inverse().toRotationMatrix(), r, p, y);
    std::cout << "================" << std::endl;
    std::cout << "rpy " << r * 54 << " " << p * 54 << " " << y * 54
              << std::endl;
    std::cout << "xyz " << tic_op(0) << " " << tic_op(1) << " " << tic_op(2)
              << std::endl;
  }
}
