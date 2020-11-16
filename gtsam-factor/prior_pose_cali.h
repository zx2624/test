#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class PriorPoseCaliFactor : public NoiseModelFactor2<Pose3, Pose3> {
private:
  typedef NoiseModelFactor2<Pose3, Pose3> Base;

  Pose3 pose_mesure_; ///< Position measurement in cartesian coordinates

public:
  /// Typedef to this class
  typedef PriorPoseCaliFactor This;

  PriorPoseCaliFactor(Key key1, Key key2,  const Pose3 &gpsIn,
                      const SharedNoiseModel &model)
      : Base(model, key1, key2), pose_mesure_(gpsIn) {}

  /// print
  void
  print(const std::string &s,
        const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector
  evaluateError(const Pose3 &p_o, const Pose3 &p_m_o,
                boost::optional<Matrix &> H1 = boost::none,
                boost::optional<Matrix &> H2 = boost::none) const override;
};
} // namespace gtsam