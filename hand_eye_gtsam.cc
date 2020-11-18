#include <ceres/ceres.h>
#include <stdarg.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <yaml-cpp/yaml.h>

#include "gtsam-factor/prior_pose_cali.h"
#include "tools/tools.h"
using namespace std;

int main(int argc, char** argv) {

  YAML::Node yn = YAML::LoadFile(argv[1]);
  double bigyawrate = yn["bigyawrate"].as<double>();
  double smallyawrate = yn["smallyawrate"].as<double>();
  double bigspeed = yn["bigspeed"].as<double>();
  double smallspeed = yn["smallspeed"].as<double>();
  double big_dis_noise = yn["big_dis_noise"].as<double>();
  double small_dis_noise = yn["small_dis_noise"].as<double>();
  double big_angle_noise = yn["big_angle_noise"].as<double>();
  double small_angle_noise = yn["small_angle_noise"].as<double>();
  double frequency = yn["frequency"].as<double>();
  int scene_size = yn["scene_size"].as<int>();
  int cali_cnt = yn["cali_cnt"].as<int>();
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> scene_frame_gen{200, 50};

  // yawrate, speed, noise
  std::normal_distribution<double> scene_change{0., bigyawrate}; // rad/s 转弯
  std::normal_distribution<double> scene_change_small{0., smallyawrate};

  std::normal_distribution<double> velocity{0, bigspeed};
  std::normal_distribution<double> velocity_small{0, smallspeed};

  std::normal_distribution<double> angle_obs{0., small_angle_noise};
  std::normal_distribution<double> dis_obs{0., small_dis_noise};

  std::normal_distribution<double> angle_obs_big{0., big_angle_noise};
  std::normal_distribution<double> dis_obs_big{0., big_dis_noise};

  for (int itercnt = 0; itercnt < cali_cnt; ++itercnt) {
    Eigen::Vector3d gyr(0.0, 0.00, 0);
    Eigen::Vector3d vel(15, 0, 0);

    Eigen::Vector3d tic(1, 1, 1);
    Eigen::Quaterniond qci(1, 0, 0, 0);
    { qci = tools::RPY2mat(0.2, 0.3, 1.57); }
    Eigen::Quaterniond qic = qci.inverse();
    Eigen::Vector3d rpy_ini = tools::mat2RPY(qci.toRotationMatrix());
    Eigen::Matrix4d Tic = tools::GetT(qic, tic);
    std::normal_distribution<double> init_r_noise{0, 0.1};
    std::normal_distribution<double> init_p_noise{0, 0.1};
    std::normal_distribution<double> init_y_noise{0, 0.2};
    std::normal_distribution<double> init_xyz_noise{0, 1};
    Eigen::Quaterniond qic_init(
        tools::RPY2mat(0.2 + init_r_noise(gen), -0.3 + init_p_noise(gen),
                       -1.2 + init_y_noise(gen))); //(1, 0, 0, 0);
    Eigen::Vector3d tic_init(init_xyz_noise(gen), init_xyz_noise(gen),
                           init_xyz_noise(gen)); //(0, 0, 0);

    // Eigen::Quaterniond qic_init = qic;
    // Eigen::Vector3d tic_init = tic;
    gtsam::Rot3 rot_ic_init(qic_init);
    gtsam::Pose3 pose_ic_init(rot_ic_init, tic_init);
    // gtsam::Pose3 pose_ic_init(Tic);
    // std::cout << "begin \n" << pose_ic_init.matrix() << std::endl;

    // isam
    gtsam::ISAM2Params isam_params;
    isam_params.factorization = gtsam::ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    gtsam::ISAM2 isam(isam_params);

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    Eigen::Matrix4d Tc0 = Eigen::Matrix4d::Identity();
    std::ofstream ofs_truth("truth" + std::to_string(itercnt) + ".txt");
    std::ofstream ofs_init("init" + std::to_string(itercnt) + ".txt");
    std::ofstream ofs_op("op" + std::to_string(itercnt) + ".txt");
    int index = -1;
    for (int i = 0; i < scene_size; ++i) {
      Eigen::Vector3d gyro_now(scene_change_small(gen), scene_change_small(gen),
                               scene_change_small(gen));
      Eigen::Vector3d vel_now =
          vel + Eigen::Vector3d(velocity(gen), velocity_small(gen),
                                velocity_small(gen));
      if (i < scene_size) { //
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
      Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
      dT.block<3, 3>(0, 0) = tools::RPY2mat(drpy(0), drpy(1), drpy(2));
      dT.block<3, 1>(0, 3) = dxyz;
      int frame_per_scene = int(scene_frame_gen(gen));
      if (frame_per_scene < 0)
        continue;
      for (int frame = 0; frame < frame_per_scene; ++frame) {
        index++;
        Eigen::Matrix4d Tc1 = Tc0 * dT;
        {
          Eigen::Vector3d rpy, xyz;
          tools::GetRPYXYZ(Tc0, rpy, xyz);
          ofs_truth << std::fixed << xyz(0) << " " << xyz(1) << " " << xyz(2)
                    << " " << rpy(0) << " " << rpy(1) << " " << rpy(2)
                    << std::endl;
        }
        // imu truth
        Eigen::Matrix4d Ti0 = Tc0 * (Tic.inverse());
        Eigen::Matrix4d Ti1 = Tc1 * (Tic.inverse());

        // can measure
        Eigen::Vector3d t_c_12(dT(0, 3) + dis_obs(gen), dT(1, 3) + dis_obs(gen),
                               dT(2, 3) + dis_obs(gen));
        Eigen::Quaterniond q_c_12(tools::RPY2mat(drpy(0) + angle_obs(gen),
                                                 drpy(1) + angle_obs(gen),
                                                 drpy(2) + angle_obs(gen)));

        gtsam::Rot3 between_rot(q_c_12.toRotationMatrix());
        gtsam::Pose3 between_pose(between_rot, t_c_12);
        // gtsam::Pose3 between_pose(dT);
        {
          // Eigen::Matrix4d mat_gtsam = gtsam::Pose3(between_rot,
          // t_c_12).matrix(); Eigen::Matrix4d mat_dt = between_pose.matrix();
          // if
          // ((dT.block<3, 3>(0, 0) - q_c_12.toRotationMatrix()).norm() > 1e-5)
          // {
          //   std::cout << "diff\n"
          //             << dT.block<3, 3>(0, 0) << "\n"
          //             << q_c_12.toRotationMatrix() << std::endl;
          // }
        }

        // gtsam::Pose3 betweennew(dT)

        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << pow(small_angle_noise, 2),
                 pow(small_angle_noise, 2), pow(small_angle_noise, 2),
                 pow(small_dis_noise, 2), pow(small_dis_noise, 2),
                 pow(small_dis_noise, 2))
                    .finished());
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Symbol('p', index), gtsam::Symbol('p', index + 1),
            between_pose, odometryNoise));

        Eigen::Vector3d rpy0_noise;
        Eigen::Vector3d rpy1_noise;
        Eigen::Vector3d t0_noise;
        Eigen::Vector3d t1_noise;
        { // imu pose measure
          Eigen::Vector3d rpy0 = tools::mat2RPY(Ti0.block<3, 3>(0, 0));
          rpy0_noise =
              rpy0 + Eigen::Vector3d(angle_obs_big(gen), angle_obs_big(gen),
                                     angle_obs_big(gen));
          Eigen::Vector3d rpy1 = tools::mat2RPY(Ti1.block<3, 3>(0, 0));
          rpy1_noise =
              rpy1 + Eigen::Vector3d(angle_obs_big(gen), angle_obs_big(gen),
                                     angle_obs_big(gen));
          Eigen::Vector3d t0 = Ti0.block<3, 1>(0, 3);
          t0_noise = t0 + Eigen::Vector3d(dis_obs_big(gen), dis_obs_big(gen),
                                          dis_obs_big(gen));
          Eigen::Vector3d t1 = Ti1.block<3, 1>(0, 3);
          t1_noise = t1 + Eigen::Vector3d(dis_obs_big(gen), dis_obs_big(gen),
                                          dis_obs(gen));
        }
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << pow(big_angle_noise, 2),
                 pow(big_angle_noise, 2), pow(big_angle_noise, 2),
                 pow(big_dis_noise, 2), pow(big_dis_noise, 2),
                 pow(big_dis_noise, 2))
                    .finished());
        if (index == 0) {
          gtsam::Rot3 rot(
              tools::RPY2mat(rpy0_noise(0), rpy0_noise(1), rpy0_noise(2)));
          gtsam::Pose3 Pose0(rot, t0_noise);
          gtSAMgraph.add(gtsam::PriorPoseCaliFactor(gtsam::Symbol('p', index),
                                                    gtsam::Symbol('c', 0),
                                                    Pose0, prior_noise));
          Eigen::Vector3d rpy_noise(angle_obs_big(gen), angle_obs_big(gen),
                                    angle_obs_big(gen));
          Eigen::Vector3d t_noise(dis_obs_big(gen), dis_obs_big(gen),
                                  dis_obs_big(gen));
          Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
          dT.block<3, 3>(0, 0) =
              tools::RPY2mat(rpy_noise(0), rpy_noise(1), rpy_noise(2));
          dT.block<3, 1>(0, 3) = t_noise;
          initialEstimate.insert(gtsam::Symbol('p', index),
                                 gtsam::Pose3(Tc0 * dT));
          initialEstimate.insert(gtsam::Symbol('c', 0), pose_ic_init);
        }
        gtsam::Rot3 rot(
            tools::RPY2mat(rpy1_noise(0), rpy1_noise(1), rpy1_noise(2)));
        gtsam::Pose3 Pose1(rot, t1_noise);
        gtSAMgraph.add(gtsam::PriorPoseCaliFactor(gtsam::Symbol('p', index + 1),
                                                  gtsam::Symbol('c', 0), Pose1,
                                                  prior_noise));
        {
          Eigen::Vector3d rpy_noise(angle_obs_big(gen), angle_obs_big(gen),
                                    angle_obs_big(gen));
          Eigen::Vector3d t_noise(dis_obs_big(gen), dis_obs_big(gen),
                                  dis_obs_big(gen));
          Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
          dT.block<3, 3>(0, 0) =
              tools::RPY2mat(rpy_noise(0), rpy_noise(1), rpy_noise(2));
          dT.block<3, 1>(0, 3) = t_noise;
          initialEstimate.insert(gtsam::Symbol('p', index + 1),
                                 gtsam::Pose3(Tc1 * dT));
          {
            Eigen::Vector3d rpy, xyz;
            tools::GetRPYXYZ(Tc1 * dT, rpy, xyz);
            ofs_init << std::fixed << xyz(0) << " " << xyz(1) << " " << xyz(2)
                     << " " << rpy(0) << " " << rpy(1) << " " << rpy(2)
                     << std::endl;
          }
        }
        if (index % 10 == 0 && index > 0) {
          isam.update(gtSAMgraph, initialEstimate);
          gtSAMgraph.resize(0);
          initialEstimate.clear();
          auto result = isam.calculateEstimate();
          auto T_ci_op =
              result.at<gtsam::Pose3>(gtsam::Symbol('c', 0)).matrix().inverse();
          Eigen::Matrix4d mat_diff = T_ci_op.inverse() * Tic.inverse();
          Eigen::Vector3d rpy_diff = tools::mat2RPY(mat_diff.block<3, 3>(0, 0));
          std::cout << "rpy: " << rpy_diff(0) << " " << rpy_diff(1) << " "
                    << rpy_diff(2) << " " << std::endl;
          std::cout << "xyz: " << mat_diff(0, 3) << " " << mat_diff(1, 3) << " "
                    << mat_diff(2, 3) << std::endl;
          for (int j = index - 9; j < index + 1; ++j) {
            auto Tj = result.at<gtsam::Pose3>(gtsam::Symbol('p', j)).matrix();
            Eigen::Vector3d rpy, xyz;
            tools::GetRPYXYZ(Tj, rpy, xyz);
            ofs_op << std::fixed << xyz(0) << " " << xyz(1) << " " << xyz(2)
                   << " " << rpy(0) << " " << rpy(1) << " " << rpy(2)
                   << std::endl;
          }
        }
        Tc0 = Tc1;
      }
    }
    // ofs_init.close();
    // ofs_truth.close();
    // ofs_op.close();
    // auto result = isam.calculateEstimate();
    // auto T_ci_op =
    //     result.at<gtsam::Pose3>(gtsam::Symbol('c', 0)).matrix().inverse();
    // Eigen::Matrix4d mat_diff = T_ci_op.inverse() * Tic.inverse();
    // Eigen::Vector3d rpy_diff = tools::mat2RPY(mat_diff.block<3, 3>(0, 0));
    // std::cout << "rpy: " << rpy_diff(0) << " " << rpy_diff(1) << " "
    //           << rpy_diff(2) << " " << std::endl;
    // std::cout << "xyz: " << mat_diff(0, 3) << " " << mat_diff(1, 3) << " "
    //           << mat_diff(2, 3) << std::endl;
    // gtsam::Values result =
    //     gtsam::LevenbergMarquardtOptimizer(gtSAMgraph, initialEstimate)
    //         .optimize();

    // // std::cout << Tic << std::endl;
    // auto mat = result.at<gtsam::Pose3>(gtsam::Symbol('c', 0)).matrix();
    // Eigen::Matrix4d mat_diff = mat.inverse() * Tic;
    // Eigen::Vector3d rpy_diff = tools::mat2RPY(mat_diff.block<3, 3>(0, 0));
    // std::cout << "rpy: " << rpy_diff(0) << " " << rpy_diff(1) << " "
    //           << rpy_diff(2) << " " << std::endl;
    // std::cout << "xyz: " << mat_diff(0, 3) << " " << mat_diff(1, 3) << " "
    //           << mat_diff(2, 3) << std::endl;
  }
  return 0;
}
