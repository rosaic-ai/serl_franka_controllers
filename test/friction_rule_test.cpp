// Unit tests for the joint friction feedforward rule (include/serl_franka_controllers/friction_rule.h).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <serl_franka_controllers/friction_rule.h>

namespace fr = serl_franka_controllers::friction;

namespace {

fr::Params hybrid() {
  fr::Params p;
  p.mode = fr::kHybrid;
  return p;
}

}  // namespace

TEST(FrictionRule, OffModeIsZero) {
  fr::Params p;
  p.mode = fr::kOff;
  double stuck = 0.3;
  EXPECT_EQ(fr::torque(1.0, 0.5, 0.5, true, 0.0, 0.001, &stuck, p), 0.0);
  EXPECT_EQ(stuck, 0.0);
}

TEST(FrictionRule, DefaultModeIsTheOriginalDesiredVelocityRule) {
  fr::Params p;  // mode 1
  double stuck = 0.0;
  for (double dq_d : {-0.2, -0.004, 0.0, 0.003, 0.05}) {
    EXPECT_EQ(fr::torque(1.3, 0.0, dq_d, false, 50.0, 0.001, &stuck, p), 1.3 * std::tanh(dq_d / 0.01));
  }
}

TEST(FrictionRule, HybridIsOffWhenFtIsUnknown) {
  double stuck = 0.2;
  EXPECT_EQ(fr::torque(1.0, 0.1, 0.1, false, 0.0, 0.001, &stuck, hybrid()), 0.0);
  EXPECT_EQ(stuck, 0.0);
}

TEST(FrictionRule, HybridFollowsMeasuredDirectionWhileMoving) {
  double stuck = 0.1;
  // moving opposite to the target (overshoot): measured direction wins, stuck timer resets
  EXPECT_DOUBLE_EQ(fr::torque(1.0, -0.05, 0.08, true, 10.0, 0.001, &stuck, hybrid()), std::tanh(-0.05 / 0.01));
  EXPECT_EQ(stuck, 0.0);
}

TEST(FrictionRule, HybridPushesAStuckJointWithoutContact) {
  double stuck = 0.0;
  EXPECT_DOUBLE_EQ(fr::torque(1.0, 0.0, 0.03, true, 1.0, 0.001, &stuck, hybrid()), std::tanh(0.03 / 0.01));
  EXPECT_DOUBLE_EQ(stuck, 0.001);
}

TEST(FrictionRule, HybridPushFadesOutAfterTheBreakawayWindow) {
  double stuck = 0.0;
  double half_way = 0.0;
  double t = 0.0;
  for (int k = 0; k < 600; ++k) {  // 0.6 s held still
    t = fr::torque(1.0, 0.0, 0.03, true, 0.0, 0.001, &stuck, hybrid());
    if (k == 449) half_way = t;  // 0.45 s: half way through the 0.1 s ramp
  }
  EXPECT_NEAR(half_way, 0.5 * std::tanh(3.0), 0.02);
  EXPECT_EQ(t, 0.0);
}

TEST(FrictionRule, HybridPushFadesWithWristForce) {
  const double full = std::tanh(0.03 / 0.01);
  double stuck = 0.0;
  EXPECT_DOUBLE_EQ(fr::torque(1.0, 0.0, 0.03, true, 1.5, 0.001, &stuck, hybrid()), full);
  stuck = 0.0;
  EXPECT_NEAR(fr::torque(1.0, 0.0, 0.03, true, 2.25, 0.001, &stuck, hybrid()), 0.5 * full, 1e-12);
  stuck = 0.0;
  EXPECT_EQ(fr::torque(1.0, 0.0, 0.03, true, 3.0, 0.001, &stuck, hybrid()), 0.0);
}

TEST(FrictionRule, HybridIsQuietWhenJointAndTargetAreStill) {
  double stuck = 0.2;
  EXPECT_EQ(fr::torque(1.0, 0.002, 0.001, true, 0.0, 0.001, &stuck, hybrid()), 0.0);
  EXPECT_EQ(stuck, 0.0);
}

TEST(FrictionRule, HybridTorqueHasNoStepAsTheJointStartsMoving) {
  // the first hybrid version dropped from the full push to a quarter at the speed threshold
  double previous = 0.0;
  double largest_step = 0.0;
  for (int k = 0; k <= 200; ++k) {
    double stuck = 0.0;
    const double dq_f = 0.0001 * k;  // 0 -> 0.02 rad/s
    const double t = fr::torque(1.0, dq_f, 0.03, true, 0.0, 0.001, &stuck, hybrid());
    if (k > 0) largest_step = std::max(largest_step, std::abs(t - previous));
    previous = t;
  }
  EXPECT_LT(largest_step, 0.05);  // < 5 % of c per 0.0001 rad/s
}

TEST(FrictionRule, LowPassConvergesAndPassesThroughWhenDisabled) {
  double y = 0.0;
  for (int k = 0; k < 200; ++k) y = fr::lowPass(y, 1.0, 20.0, 0.001);
  EXPECT_NEAR(y, 1.0, 1e-6);
  EXPECT_EQ(fr::lowPass(0.3, 0.7, 0.0, 0.001), 0.7);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
