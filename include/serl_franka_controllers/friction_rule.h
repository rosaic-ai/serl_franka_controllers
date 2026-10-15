// Joint friction feedforward rule (one joint, one control tick).
//
// Mode 1 (default) is the original rule: Coulomb torque along the desired joint
// velocity. It helps a joint held by static friction start, but keeps pushing
// when the hand is blocked by an object (target moves, joint does not).
//
// Mode 2 (hybrid, 2026-10-15) tells the two "joint still, target moving" cases
// apart with the relayed wrist force, and blends every hand-over so the torque
// never steps (the first hybrid version switched branches at a speed threshold
// and doubled the torque roughness in the free-space test):
//   - moving part : along the low-pass filtered measured velocity
//   - stuck part  : along the desired velocity, faded out as the wrist force
//                   nears contact_n and as the joint stays stuck past breakaway_s
//   - the weight between the two follows the filtered speed from v_move/2 to v_move
//   - F/T relay unknown/stale -> 0 (fail-closed)
// No ROS or Eigen dependency, so it is unit-tested on its own (test/friction_rule_test.cpp).

#pragma once

#include <algorithm>
#include <cmath>

namespace serl_franka_controllers {
namespace friction {

enum Mode { kOff = 0, kDesired = 1, kHybrid = 2 };

struct Params {
  int mode{kDesired};
  double v0_desired{0.01};       // rad/s, tanh width along the desired velocity
  double v_move{0.005};          // rad/s, filtered speed at which a joint counts as fully moving
  double v0_actual{0.01};        // rad/s, tanh width along the filtered measured velocity
  double breakaway_s{0.4};       // s, how long a stuck joint gets the full desired-direction push
  double breakaway_ramp_s{0.1};  // s, fade-out of that push after breakaway_s
  double contact_n{3.0};         // N, relayed wrist force at which the push is gone (fades from contact_n/2)
};

inline double clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

// Returns the feedforward torque (Nm) for one joint.
//   c: friction value (Nm); dq_f: low-pass filtered measured joint velocity; dq_d: desired joint velocity (rad/s);
//   ft_known: a fresh relayed F/T value exists; ft_magnitude: its |F| (N);
//   dt: control period (s); stuck_s: per-joint time held still while the target moves (updated).
inline double torque(double c, double dq_f, double dq_d, bool ft_known, double ft_magnitude,
                     double dt, double* stuck_s, const Params& p) {
  if (p.mode == kDesired) {
    *stuck_s = 0.0;
    return c * std::tanh(dq_d / p.v0_desired);
  }
  if (p.mode != kHybrid || !ft_known) {
    *stuck_s = 0.0;
    return 0.0;
  }
  const double moving = clamp01((std::abs(dq_f) - 0.5 * p.v_move) / (0.5 * p.v_move));
  const bool target_moving = std::abs(dq_d) > p.v_move;
  *stuck_s = (target_moving && moving < 1.0) ? *stuck_s + dt : 0.0;

  const double moving_part = c * std::tanh(dq_f / p.v0_actual);
  const double contact_gate = clamp01((p.contact_n - ft_magnitude) / (0.5 * p.contact_n));
  const double time_gate =
      p.breakaway_ramp_s > 0.0 ? clamp01((p.breakaway_s + p.breakaway_ramp_s - *stuck_s) / p.breakaway_ramp_s)
                               : (*stuck_s < p.breakaway_s ? 1.0 : 0.0);
  const double stuck_part = target_moving ? c * std::tanh(dq_d / p.v0_desired) * contact_gate * time_gate : 0.0;
  return moving * moving_part + (1.0 - moving) * stuck_part;
}

// One step of the first-order low-pass filter that makes dq_f (cutoff_hz <= 0 passes the sample through).
inline double lowPass(double previous, double sample, double cutoff_hz, double dt) {
  if (cutoff_hz <= 0.0) {
    return sample;
  }
  const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt);
  return previous + alpha * (sample - previous);
}

}  // namespace friction
}  // namespace serl_franka_controllers
