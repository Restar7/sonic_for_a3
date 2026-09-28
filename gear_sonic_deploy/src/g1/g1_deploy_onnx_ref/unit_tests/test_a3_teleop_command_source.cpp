// Standalone unit test for the teleop command bridge (plan sections 59/60):
// A3_REFERENCE_V1 window -> /ta/whole_body_command field groups.
//
// It needs no AimRT and no ZMQ: the point is the joint-order permutation, the
// validation and the monotonicity/staleness behaviour.  Build/run with:
//
//   g++ -std=c++17 -I include unit_tests/test_a3_teleop_command_source.cpp
//           src/a3_deploy/a3_teleop_command_source.cpp -o /tmp/test_teleop_cmd
//   /tmp/test_teleop_cmd

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "a3_deploy/a3_teleop_command_source.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const std::string& what) {
  if (condition) {
    std::printf("  [ OK ] %s\n", what.c_str());
  } else {
    std::printf("  [FAIL] %s\n", what.c_str());
    ++g_failures;
  }
}

// Order documented by ConvertTaWholeBodyCommand in a3_teleop_reference.cpp.
const char* const kExpectedPolicyOrder[29] = {
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint"};

// The official converter, re-implemented here so the test can round-trip our
// fields back into the A3TeleopFrame the tokenizer actually sees.
struct RoundTripFrame {
  double q_policy[29];
  double dq_policy[29];
  double pelvis_quat[4];
  bool has_head = false;
};

RoundTripFrame ApplyOfficialConverter(const a3_deploy::A3WholeBodyCommandFields& f) {
  RoundTripFrame out{};
  for (int i = 0; i < 3; ++i) out.q_policy[i] = f.waist_angles_rad[i];
  for (int i = 0; i < 14; ++i) out.q_policy[3 + i] = f.arm_angles_rad[i];
  for (int i = 0; i < 12; ++i) out.q_policy[17 + i] = f.leg_angles_rad[i];
  for (int i = 0; i < 3; ++i) out.dq_policy[i] = f.velocities_rad_s[12 + i];
  for (int i = 0; i < 14; ++i) out.dq_policy[3 + i] = f.velocities_rad_s[16 + i];
  for (int i = 0; i < 12; ++i) out.dq_policy[17 + i] = f.velocities_rad_s[i];
  for (int i = 0; i < 4; ++i) out.pelvis_quat[i] = f.pelvis_quat_wxyz[i];
  out.has_head = f.has_head_command;
  return out;
}

a3_deploy::A3ReferenceWindow MakeWindow(std::uint64_t seq, std::int64_t stamp_ns, double scale) {
  a3_deploy::A3ReferenceWindow w{};
  w.valid = true;
  w.seq = seq;
  w.timestamp_ns = stamp_ns;
  w.dt = 0.02f;
  for (int f = 0; f < a3_deploy::kA3RefFrames; ++f) {
    for (int j = 0; j < a3_deploy::kA3RefJoints; ++j) {
      // marker value: joint index (fractional so frames differ)
      w.joint_pos_rad[f * a3_deploy::kA3RefJoints + j] =
          static_cast<float>(scale * (j + 0.001 * f));
      w.joint_vel_rad_s[f * a3_deploy::kA3RefJoints + j] =
          static_cast<float>(scale * (0.1 * j + 0.001 * f));
    }
    w.root_pos_m[f * 3 + 0] = 0.0f;
    w.root_pos_m[f * 3 + 1] = 0.0f;
    w.root_pos_m[f * 3 + 2] = 1.07f;
    w.root_quat_wxyz[f * 4 + 0] = 1.0f;
    w.root_quat_wxyz[f * 4 + 1] = 0.0f;
    w.root_quat_wxyz[f * 4 + 2] = 0.0f;
    w.root_quat_wxyz[f * 4 + 3] = 0.0f;
  }
  return w;
}

// Keep every marker value inside the real limits.
void ClampToLimits(a3_deploy::A3ReferenceWindow* w) {
  for (int f = 0; f < a3_deploy::kA3RefFrames; ++f) {
    for (int j = 0; j < a3_deploy::kA3RefJoints; ++j) {
      const double lo = a3_deploy::kA3ReferenceLimits.position_lower[j];
      const double hi = a3_deploy::kA3ReferenceLimits.position_upper[j];
      const int idx = f * a3_deploy::kA3RefJoints + j;
      const double v = w->joint_pos_rad[idx];
      const double mid = 0.5 * (lo + hi);
      const double half = 0.25 * (hi - lo);
      w->joint_pos_rad[idx] = static_cast<float>(mid + half * std::tanh(v == 0.0 ? 0.0 : (v > 0 ? 1.0 : -1.0)));
    }
  }
}

}  // namespace

int main() {
  using namespace a3_deploy;

  std::printf("== joint order table ==\n");
  bool names_match = true;
  for (int i = 0; i < 29; ++i) {
    if (std::string(kA3PolicyJointNames[i]) != kExpectedPolicyOrder[i]) names_match = false;
  }
  Check(names_match, "kA3PolicyJointNames matches ConvertTaWholeBodyCommand policy order");
  bool bijection = true;
  for (int i = 0; i < 29; ++i) {
    if (kA3IlToPolicyIndex[kA3PolicyToIlIndex[i]] != i) bijection = false;
  }
  Check(bijection, "policy<->il permutations are exact inverses");
  bool names_agree = true;
  for (int p = 0; p < 29; ++p) {
    if (std::string(kA3PolicyJointNames[p]) != kA3IlJointNames[kA3PolicyToIlIndex[p]]) {
      names_agree = false;
    }
  }
  Check(names_agree, "policy name == il name at the mapped index");

  std::printf("== window -> command fields ==\n");
  A3ReferenceWindow window = MakeWindow(1, 1'000'000'000LL, 0.3);
  ClampToLimits(&window);
  A3WholeBodyCommandFields fields{};
  std::string error;
  Check(BuildWholeBodyCommandFromReference(window, 0, 1'000'000'000LL, &fields, &error),
        "converts frame 0 (" + error + ")");
  const RoundTripFrame rt = ApplyOfficialConverter(fields);
  bool wire_round_trip = true;
  for (int il = 0; il < 29; ++il) {
    const int policy = kA3IlToPolicyIndex[il];
    if (std::fabs(rt.q_policy[policy] - window.joint_pos_rad[il]) > 1e-6) wire_round_trip = false;
  }
  Check(wire_round_trip,
        "every wire joint index lands on the policy index the official converter reads");
  Check(!rt.has_head, "no head command is claimed (A3_REFERENCE_V1 carries no head)");
  Check(std::fabs(rt.pelvis_quat[0] - 1.0) < 1e-9, "pelvis quaternion is forwarded");

  // velocity layout: leg(12) + waist(3) + head(1) + arm(14)
  bool vel_round_trip = true;
  for (int il = 0; il < 29; ++il) {
    const int policy = kA3IlToPolicyIndex[il];
    if (std::fabs(rt.dq_policy[policy] - window.joint_vel_rad_s[il]) > 1e-6) vel_round_trip = false;
  }
  Check(vel_round_trip, "velocities round-trip through the 30-slot protocol layout");

  std::printf("== rejection paths ==\n");
  A3ReferenceWindow broken = window;
  broken.joint_pos_rad[7] = std::nanf("");
  Check(!BuildWholeBodyCommandFromReference(broken, 0, 1'000'000'000LL, &fields, &error),
        "NaN joint rejected");
  broken = window;
  broken.valid = false;
  Check(!BuildWholeBodyCommandFromReference(broken, 0, 1'000'000'000LL, &fields, &error),
        "invalid window rejected");
  Check(!BuildWholeBodyCommandFromReference(window, 99, 1'000'000'000LL, &fields, &error),
        "out-of-range frame index rejected");

  std::printf("== pump: emission, monotonic stamps, dedup ==\n");
  A3TeleopCommandPumpOptions options;
  options.max_joint_step_rad = 10.0;  // marker motion is artificial
  A3TeleopCommandPump pump(options);
  std::vector<A3WholeBodyCommandFields> out;
  const std::int64_t receive = 5'000'000'000LL;
  Check(pump.PushWindow(window, receive, &out, &error), "first window emits commands");
  Check(out.size() == static_cast<std::size_t>(kA3RefFrames),
        "all 10 window frames are emitted (dense 50 Hz reference)");
  bool monotonic = true;
  for (std::size_t i = 1; i < out.size(); ++i) {
    if (out[i].stamp_ns <= out[i - 1].stamp_ns) monotonic = false;
  }
  Check(monotonic, "emitted timestamps are strictly increasing");
  const std::int64_t expected_offset = receive - window.timestamp_ns;
  Check(pump.ClockOffsetNs() == expected_offset, "clock offset calibrated from the first window");
  Check(std::llabs(out.front().stamp_ns - receive) <= 1,
        "the first emitted frame is stamped with the receive time");

  // overlapping window: frames already pushed must not be re-published
  A3ReferenceWindow next = MakeWindow(2, window.timestamp_ns + 100'000'000LL, 0.3);
  ClampToLimits(&next);
  const std::size_t before = pump.Stats().frames_emitted;
  pump.PushWindow(next, receive + 100'000'000LL, &out, &error);
  Check(pump.Stats().frames_skipped_stale_stamp >= 5,
        "frames that were already published are skipped (no duplicate commands)");
  Check(pump.Stats().frames_emitted > before, "the fresh part of the window is still published");

  // non-monotonic sequence
  const std::uint64_t accepted_before = pump.Stats().windows_accepted;
  Check(!pump.PushWindow(window, receive + 200'000'000LL, &out, &error),
        "older/duplicate window sequence rejected");
  Check(pump.Stats().windows_accepted == accepted_before, "rejected window is not counted as accepted");

  std::printf("== pump: staleness gate ==\n");
  Check(!pump.ShouldHold(receive + 10'000'000LL), "fresh window -> keep publishing");
  Check(pump.ShouldHold(receive + 1'000'000'000LL),
        "no window for 1 s -> hold (runtime watchdog then safe-halts)");

  std::printf("== pump: teleport guard ==\n");
  A3TeleopCommandPumpOptions guard_options;
  guard_options.max_joint_step_rad = 0.01;
  guard_options.emit_all_window_frames = false;
  A3TeleopCommandPump guarded(guard_options);
  guarded.PushWindow(window, receive, &out, &error);
  A3ReferenceWindow jumped = window;
  for (int j = 0; j < kA3RefJoints; ++j) {
    const double lo = kA3ReferenceLimits.position_lower[j];
    const double hi = kA3ReferenceLimits.position_upper[j];
    jumped.joint_pos_rad[j] = static_cast<float>(0.5 * (lo + hi));
  }
  jumped.seq = 9;
  jumped.timestamp_ns = window.timestamp_ns + 100'000'000LL;
  Check(!guarded.PushWindow(jumped, receive + 100'000'000LL, &out, &error),
        "a teleport between consecutive ticks is rejected");
  Check(guarded.Stats().rejected_step == 1, "teleport counted separately");

  std::printf("\n%s (%d failure(s))\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
  return g_failures == 0 ? 0 : 1;
}
