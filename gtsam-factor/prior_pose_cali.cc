#include "prior_pose_cali.h"

namespace gtsam {
void PriorPoseCaliFactor::print(const std::string &s,
                                const KeyFormatter &keyFormatter) const {
  cout << s << "PriorPoseCaliFactor on " << keyFormatter(key()) << "\n";
  cout << "  PriorPoseCaliFactor measurement: " << pose_mesure_ << "\n";
  noiseModel_->print("  noise model: ");
}

bool PriorPoseCaliFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Pose3>::Equals(pose_mesure_, e->pose_mesure_, tol);
}
Vector PriorPoseCaliFactor::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  return p.translation(H) -nT_;
}
} // namespace gtsam