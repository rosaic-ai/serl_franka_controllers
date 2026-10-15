// Joint friction feedforward rule (one joint, one control tick).
//
// Mode 1 (default) is the original rule: Coulomb torque along the desired joint
// velocity. It helps a joint held by static friction start, but keeps pushing
// when the hand is blocked by an object (target moves, joint does not).
//
// Mode 2 (hybrid, 2026-10-15) tells the two "joint still, target moving" cases
// apart with the relayed wrist force:
//   - joint moving             -> along the measured velocity (only real sliding friction)
//   - still, target moving, no wrist force, still for < breakaway_s
//                              -> along the desired velocity (breakaway push)
//   - still, target moving, wrist force or longer
//                              -> 0 (blocked: extra torque would only become contact force)
//   - both still               -> measured-velocity rule (near zero)
//   - F/T relay unknown/stale  -> 0 (fail-closed)
// No ROS or Eigen dependency, so it is unit-tested on its own (test/friction_rule_test.cpp).

#pragma once

#include <cmath>

namespace serl_franka_controllers {
namespace friction {

enum Mode { kOff = 0, kDesired = 1, kHybrid = 2 };

struct Params {
  int mode{kDesired};
  double v0_desired{0.01};   // rad/s, tanh width along the desired velocity
  double v_move{0.005};      // rad/s, above this a joint counts as moving
  double v0_actual{0.02};    // rad/s, tanh width along the measured velocity
  double breakaway_s{0.4};   // s, longest desired-direction push for a stuck joint
  double contact_n{3.0};     // N, relayed wrist force at which a stuck joint counts as blocked
};

// Returns the feedforward torque (Nm) for one joint.
//   c: friction value (Nm); dq, dq_d: measured and desired joint velocity (rad/s);
//   ft_known: a fresh relayed F/T value exists; ft_magnitude: its |F| (N);
//   dt: control period (s); stuck_s: per-joint time held still while the target moves (updated).
inline double torque(double c, double dq, double dq_d, bool ft_known, double ft_magnitude,
                     double dt, double* stuck_s, const Params& p) {
  if (p.mode == kDesired) {
    *stuck_s = 0.0;
    return c * std::tanh(dq_d / p.v0_desired);
  }
  if (p.mode != kHybrid || !ft_known) {
    *stuck_s = 0.0;
    return 0.0;
  }
  if (std::abs(dq) > p.v_move) {
    *stuck_s = 0.0;
    return c * std::tanh(dq / p.v0_actual);
  }
  if (std::abs(dq_d) > p.v_move) {
    *stuck_s += dt;
    const bool breakaway = ft_magnitude < p.contact_n && *stuck_s < p.breakaway_s;
    return breakaway ? c * std::tanh(dq_d / p.v0_desired) : 0.0;
  }
  *stuck_s = 0.0;
  return c * std::tanh(dq / p.v0_actual);
}

}  // namespace friction
}  // namespace serl_franka_controllers
