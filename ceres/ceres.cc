#include<iostream>
#include<Eigen/Core>
#include<ceres/ceres.h>

class TestTerm {
 public:
  TestTerm() {
    q = (Eigen::AngleAxis<double>(0.1, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxis<double>(0.02, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<double>(0.03, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  }
  

 private:
  Eigen::Quaterniond q;
};