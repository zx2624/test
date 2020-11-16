#include <ceres/ceres.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

struct HandEye {
  HandEye(Eigen::Quaterniond& q_i_12, Eigen::Vector3d& t_i_12,
          Eigen::Quaterniond& q_c_12, Eigen::Vector3d& t_c_12)
      : q_i_12_(q_i_12), t_i_12_(t_i_12), q_c_12_(q_c_12), t_c_12_(t_c_12) {}
  template <typename T>
  bool operator()(const T* const q_ic_ptr, const T* const t_ic_ptr,
                  T* res) const {
    Eigen::Quaternion<T> qic(q_ic_ptr[3], q_ic_ptr[0], q_ic_ptr[1],
                             q_ic_ptr[2]);
    Eigen::Matrix<T, 3, 1> tic(t_ic_ptr[0], t_ic_ptr[1], t_ic_ptr[2]);
    Eigen::Matrix<T, 4, 4> Tic = Eigen::Matrix<T, 4, 4>::Identity();
    Tic.template block<3, 3>(0, 0) = qic.toRotationMatrix();
    Tic.template block<3, 1>(0, 3) = tic;
    Eigen::Matrix<T, 4, 4> T_i_12 = Eigen::Matrix<T, 4, 4>::Identity();
    T_i_12.template block<3, 3>(0, 0) =
        (q_i_12_.template cast<T>()).toRotationMatrix();
    T_i_12.template block<3, 1>(0, 3) = t_i_12_.template cast<T>();

    Eigen::Matrix<T, 4, 4> T_c_12 = Eigen::Matrix<T, 4, 4>::Identity();
    T_c_12.template block<3, 3>(0, 0) =
        (q_c_12_.template cast<T>()).toRotationMatrix();
    T_c_12.template block<3, 1>(0, 3) = t_c_12_.template cast<T>();

    Eigen::Matrix<T, 4, 4> T_err = (Tic * T_c_12).inverse() * (T_i_12 * Tic);
    Eigen::Quaternion<T> q_err(T_err.template block<3, 3>(0, 0));
    res[0] = q_err.x() / T(0.005);
    res[1] = q_err.y() / T(0.005);
    res[2] = q_err.z() / T(0.005);
    res[3] = T_err(0, 3) / T(0.5);
    res[4] = T_err(1, 3) / T(0.5);
    res[5] = T_err(2, 3) / T(0.5);
    return true;
  }

  Eigen::Quaterniond q_i_12_;
  Eigen::Vector3d t_i_12_;
  Eigen::Quaterniond q_c_12_;
  Eigen::Vector3d t_c_12_;
};

struct RotateV {
  RotateV() {}
  template <typename T>
  bool operator()(const T* const q, const T* const t_ptr, const T* const v,
                  T* res) const {
    Eigen::Quaternion<T> qic(q[3], q[0], q[1], q[2]);
    Eigen::Matrix<T, 3, 1> t(t_ptr[0], t_ptr[1], t_ptr[2]);
    Eigen::Matrix<T, 3, 1> vec(v[0], v[1], v[2]);
    Eigen::Matrix<T, 3, 1> verr = qic * vec + t;
    res[0] = verr(0);
    res[1] = verr(1);
    res[2] = verr(2);
    return true;
  }
};
template <typename T>
Eigen::Matrix<T, 3, 3> unsym_mat(Eigen::Matrix<T, 3, 1> vec) {
  Eigen::Matrix<T, 3, 3> mat;
  mat << T(0.), T(-1.) * vec(2), vec(1), vec(2), T(0.), T(-1.) * vec(0),
      T(-1.) * vec(1), vec(0), T(0.);
  return mat;
}
struct Skew2V {
  Skew2V() {}
  template <typename T>
  bool operator()(const T* theta, const T* v, T* res) const {
    Eigen::Matrix<T, 3, 3> mat_theta =
        unsym_mat(Eigen::Matrix<T, 3, 1>(theta[0], theta[1], theta[2]));
    Eigen::Matrix<T, 3, 3> mat_theta2 = mat_theta * mat_theta;
    Eigen::Matrix<T, 3, 1> vec(v[0], v[1], v[2]);
    Eigen::Matrix<T, 3, 1> verr = mat_theta2 * vec;
    res[0] = verr(0);
    res[1] = verr(1);
    res[2] = verr(2);

    return true;
  }
  // {
  //   ceres::Problem prob;
  //   ceres::CostFunction* cost =
  //       new ceres::AutoDiffCostFunction<Skew2V, 3, 3, 3>(new Skew2V);
  //   Eigen::Vector3d theta(1, 2, 3);
  //   Eigen::Vector3d v(4, 5, 6);
  //   // std::cout << unsym_mat(theta) * unsym_mat(v) << std::endl;
  //   // std::cout << unsym_mat(Eigen::Vector3d(unsym_mat(theta) * v)) <<
  //   // std::endl;
  //   double jaco_theta[9];
  //   double jaco_v[9];
  //   double* jacobian[] = {jaco_theta, jaco_v};
  //   double* params[] = {theta.data(), v.data()};
  //   double res[10];
  //   cost->Evaluate(params, res, jacobian);
  //   Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jaco_eigen(
  //       jaco_theta);
  //   std::cout << "   theta: " << std::endl;
  //   std::cout << jaco_eigen << std::endl;

  //   Eigen::Matrix3d mat_theta = unsym_mat(theta);
  //   Eigen::Matrix3d mat_v = unsym_mat(v);
  //   std::cout << unsym_mat(Eigen::Vector3d(mat_theta * v)) + mat_theta * mat_v
  //             << std::endl;

  //   std::cout << "   v: " << std::endl;
  //   Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jaco_v_eigen(
  //       jaco_v);
  //   std::cout << jaco_v_eigen << std::endl;

  //   std::cout << mat_theta * mat_theta << std::endl;
  // }
};