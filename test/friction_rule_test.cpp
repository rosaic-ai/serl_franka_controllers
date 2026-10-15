// Unit tests for the joint friction feedforward rule (include/serl_franka_controllers/friction_rule.h).
#include <gtest/gtest.h>

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
  EXPECT_DOUBLE_EQ(fr::torque(1.0, -0.05, 0.08, true, 10.0, 0.001, &stuck, hybrid()), std::tanh(-0.05 / 0.02));
  EXPECT_EQ(stuck, 0.0);
}

TEST(FrictionRule, HybridPushesAStuckJointBrieflyWithoutContact) {
  double stuck = 0.0;
  const double t = fr::torque(1.0, 0.0, 0.03, true, 1.0, 0.001, &stuck, hybrid());
  EXPECT_DOUBLE_EQ(t, std::tanh(0.03 / 0.01));
  EXPECT_DOUBLE_EQ(stuck, 0.001);
}

TEST(FrictionRule, HybridStopsPushingAfterTheBreakawayWindow) {
  double stuck = 0.0;
  double t = 0.0;
  for (int k = 0; k < 450; ++k) {  // 0.45 s held still
    t = fr::torque(1.0, 0.001, 0.03, true, 0.0, 0.001, &stuck, hybrid());
  }
  EXPECT_EQ(t, 0.0);
}

TEST(FrictionRule, HybridDoesNotPushAStuckJointInContact) {
  double stuck = 0.0;
  EXPECT_EQ(fr::torque(1.0, 0.0, 0.03, true, 3.0, 0.001, &stuck, hybrid()), 0.0);
}

TEST(FrictionRule, HybridWithJointAndTargetStillUsesTheMeasuredRule) {
  double stuck = 0.2;
  EXPECT_DOUBLE_EQ(fr::torque(1.0, 0.002, 0.001, true, 0.0, 0.001, &stuck, hybrid()), std::tanh(0.002 / 0.02));
  EXPECT_EQ(stuck, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
