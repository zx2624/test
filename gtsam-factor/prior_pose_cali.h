#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class PriorPoseCaliFactor : public NoiseModelFactor1<Pose3> {
private:
  typedef NoiseModelFactor1<Pose3> Base;

  Pose3 pose_mesure_; ///< Position measurement in cartesian coordinates

public:
  /// Typedef to this class
  typedef PriorPoseCaliFactor This;

  PriorPoseCaliFactor(Key key, const Point3 &gpsIn,
                      const SharedNoiseModel &model)
      : Base(model, key), pose_mesure_(gpsIn) {}

  /// print
  void
  print(const std::string &s,
        const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector
  evaluateError(const Pose3 &p,
                boost::optional<Matrix &> H = boost::none) const override;
};
} // namespace gtsam