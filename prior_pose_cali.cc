#include "gtsam-factor/prior_pose_cali.h"

namespace gtsam {
void PriorPoseCaliFactor::print(const std::string &s,
                                const KeyFormatter &keyFormatter) const {
  cout << s << "PriorPoseCaliFactor on " << keyFormatter(key1()) << " "
       << keyFormatter(key2()) << "\n";
  cout << "  PriorPoseCaliFactor measurement: " << pose_mesure_ << "\n";
  noiseModel_->print("  noise model: ");
}

bool PriorPoseCaliFactor::equals(const NonlinearFactor &expected,
                                 double tol) const {
  const This *e = dynamic_cast<const This *>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Pose3>::Equals(pose_mesure_, e->pose_mesure_, tol);
}
Vector PriorPoseCaliFactor::evaluateError(const Pose3 &p_c, const Pose3 &p_i_c,
                                          boost::optional<Matrix &> H1,
                                          boost::optional<Matrix &> H2) const {
  Matrix66 D_hat_pic;
  Matrix66 D_hat_mes;
  Pose3 p_c_hat =
      traits<Pose3>::Compose(pose_mesure_, p_i_c, &D_hat_mes, &D_hat_pic);
  Matrix66 D_v_c, D_v_hat;
  Vector v = traits<Pose3>::Local(p_c, p_c_hat, &D_v_c, &D_v_hat);
  if (H1) {
    *H1 = D_v_c;
  }
  if (H2)
    *H2 = D_v_hat * D_hat_pic;
  return v;
}
} // namespace gtsam