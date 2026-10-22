#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Core>

namespace serl_franka_controllers {
// Additional Cartesian force only: this is NOT a total-force safety controller.
// No allocation, ROS calls or callback-owned state in the real-time path.
struct TrackingCorrectionConfig {
  double gain{600.0};          // N / (m s)
  double max_force{15.0};      // N, Euclidean norm of added force
  double slew{3.0};            // N/s, Euclidean norm of force change
  double max_error{0.015};     // m; do not integrate large pose transitions
  double max_target_speed{0.003}; // m/s
  double max_actual_speed{0.015}; // m/s
  double settle_time{0.15};    // s below speed gates before integration
  double external_force_trip{12.0}; // N, model-estimated external force
};
class BoundedTrackingCorrection {
 public:
  explicit BoundedTrackingCorrection(TrackingCorrectionConfig c = {}) : c_(c) {}
  void reset() { force_.setZero(); settled_=0.; tripped_=false; state_=0; }
  void clearTrip() { tripped_=false; settled_=0.; }
  const Eigen::Vector3d& force() const { return force_; }
  bool tripped() const { return tripped_; }
  int state() const { return state_; } // 0 off, 1 integrating, 2 gated, 3 tripped
  Eigen::Vector3d update(double dt, const Eigen::Vector3d& target_minus_actual,
                         double target_speed, double actual_speed,
                         double external_force, bool enabled) {
    // Invalid timing must not produce a large integration or release jump.
    if (!std::isfinite(dt) || dt<=0. || dt>0.005) return force_;
    const bool finite=target_minus_actual.allFinite() && std::isfinite(target_speed)
      && std::isfinite(actual_speed) && std::isfinite(external_force);
    if (enabled && (!finite || external_force>=c_.external_force_trip)) tripped_=true;
    const bool gate=finite && target_minus_actual.norm()<=c_.max_error
      && target_speed<=c_.max_target_speed && actual_speed<=c_.max_actual_speed;
    if (enabled && !tripped_ && gate) settled_+=dt; else settled_=0.;
    Eigen::Vector3d wanted=Eigen::Vector3d::Zero();
    if (enabled && !tripped_ && gate) {
      wanted=force_;
      if (settled_>=c_.settle_time) {
        wanted+=c_.gain*target_minus_actual*dt;
        state_=1;
      } else state_=2;
    } else state_=tripped_ ? 3 : (enabled ? 2 : 0);
    // Project before committing: no unobservable windup behind the force cap.
    if (wanted.norm()>c_.max_force) wanted*=c_.max_force/wanted.norm();
    Eigen::Vector3d delta=wanted-force_;
    const double limit=c_.slew*dt;
    if (delta.norm()>limit) delta*=limit/delta.norm();
    force_+=delta;
    return force_;
  }
 private:
  TrackingCorrectionConfig c_;
  Eigen::Vector3d force_{Eigen::Vector3d::Zero()};
  double settled_{0.};
  bool tripped_{false};
  int state_{0};
};
} // namespace serl_franka_controllers
