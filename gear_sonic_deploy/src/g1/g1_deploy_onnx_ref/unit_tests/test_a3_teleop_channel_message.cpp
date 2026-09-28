// Proto-level test for the teleop command bridge (plan sections 59/60).
//
// Unlike test_a3_teleop_command_source.cpp this one uses the **real generated
// protobuf code** from proto/aimdk/protocol/ta/, so it proves the message layout
// the A3 runtime's ConvertTaWholeBodyCommand reads back:
//
//   window -> fields -> TaWholeBodyCommandChannel -> (mirror of the official
//   converter) -> policy-view joints == the window's joints
//
// It needs protoc + libprotobuf, but no AimRT:
//   bash tools/run_cpp_channel_message_test.sh

#include <cmath>
#include <cstdio>
#include <string>

#include "a3_deploy/a3_teleop_channel_message.hpp"
#include "a3_deploy/a3_teleop_command_source.hpp"
#include "aimdk/protocol/ta/ta_channel.pb.h"
#include "aimdk/protocol/ta/ta_whole_body_command.pb.h"

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

a3_deploy::A3ReferenceWindow MakeWindow(std::int64_t stamp_ns) {
  a3_deploy::A3ReferenceWindow w{};
  w.valid = true;
  w.seq = 42;
  w.timestamp_ns = stamp_ns;
  w.dt = 0.02f;
  for (int f = 0; f < a3_deploy::kA3RefFrames; ++f) {
    for (int j = 0; j < a3_deploy::kA3RefJoints; ++j) {
      const double lo = a3_deploy::kA3ReferenceLimits.position_lower[j];
      const double hi = a3_deploy::A3ReferenceLimits{}.position_upper[j];
      const double mid = 0.5 * (lo + hi);
      const double half = 0.2 * (hi - lo);
      const double frac = static_cast<double>(j) / a3_deploy::kA3RefJoints;
      w.joint_pos_rad[f * a3_deploy::kA3RefJoints + j] =
          static_cast<float>(mid + half * (2.0 * frac - 1.0));
      w.joint_vel_rad_s[f * a3_deploy::kA3RefJoints + j] = static_cast<float>(0.01 * (j + f));
    }
    w.root_pos_m[f * 3 + 0] = 0.1f * f;
    w.root_pos_m[f * 3 + 1] = 0.0f;
    w.root_pos_m[f * 3 + 2] = 1.07f;
    w.root_quat_wxyz[f * 4 + 0] = 1.0f;
    w.root_quat_wxyz[f * 4 + 1] = 0.0f;
    w.root_quat_wxyz[f * 4 + 2] = 0.0f;
    w.root_quat_wxyz[f * 4 + 3] = 0.0f;
  }
  return w;
}

// Mirror of the velocity branch of ConvertTaWholeBodyCommand: the runtime picks
// the offsets from the array length, so the test must do the same.
void MirrorOfficialVelocityLayout(const google::protobuf::RepeatedField<double>& vel,
                                  double dq_policy[29]) {
  for (int i = 0; i < 29; ++i) dq_policy[i] = 0.0;
  if (vel.size() == 30) {
    // leg(12) + waist(3) + head(1) + arm(14)
    for (int i = 0; i < 3; ++i) dq_policy[i] = vel.Get(12 + i);
    for (int i = 0; i < 14; ++i) dq_policy[3 + i] = vel.Get(16 + i);
    for (int i = 0; i < 12; ++i) dq_policy[17 + i] = vel.Get(i);
  } else if (vel.size() == 31) {
    // leg(12) + waist(3) + head(2) + arm(14)
    for (int i = 0; i < 3; ++i) dq_policy[i] = vel.Get(12 + i);
    for (int i = 0; i < 14; ++i) dq_policy[3 + i] = vel.Get(17 + i);
    for (int i = 0; i < 12; ++i) dq_policy[17 + i] = vel.Get(i);
  } else if (vel.size() == 29) {
    for (int i = 0; i < 29; ++i) dq_policy[i] = vel.Get(i);
  }
}

}  // namespace

int main() {
  using namespace a3_deploy;

  const std::int64_t stamp_ns = 1'700'000'123'456'789'000LL;
  A3ReferenceWindow window = MakeWindow(stamp_ns);

  A3WholeBodyCommandFields fields{};
  std::string error;
  Check(BuildWholeBodyCommandFromReference(window, 0, stamp_ns, &fields, &error),
        "window frame -> channel fields (" + error + ")");

  aimdk::protocol::TaWholeBodyCommandChannel msg;
  FillWholeBodyCommandChannel(fields, &msg, 42, true);

  std::printf("== message shape ==\n");
  Check(msg.header().seq() == 42u, "header.seq forwarded");
  Check(msg.header().timestamp().seconds() == stamp_ns / 1'000'000'000LL,
        "header.timestamp.seconds forwarded");
  Check(msg.header().timestamp().nanos() == static_cast<int>(stamp_ns % 1'000'000'000LL),
        "header.timestamp.nanos forwarded");
  Check(msg.data().joint_layout() == aimdk::protocol::TaJointLayout_BODY_31,
        "joint_layout = BODY_31");
  Check(msg.data().leg_command().angles_rad_size() == 12, "leg command has 12 entries");
  Check(msg.data().waist_command().angles_rad_size() == 3, "waist command has 3 entries");
  Check(msg.data().arm_command().angles_rad_size() == 14, "arm command has 14 entries");
  Check(msg.data().joint_velocities().velocities_rad_s_size() == 31,
        "velocity layout is leg12 + waist3 + head2 + arm14 = 31");
  Check(!msg.data().has_head_command(),
        "no head command is sent (A3_REFERENCE_V1 carries no head data)");
  Check(msg.data().pelvis_pose().quat_wxyz_size() == 4, "pelvis quaternion has 4 entries");
  Check(msg.data().pelvis_pose().position_xyz_size() == 3, "pelvis position has 3 entries");

  std::printf("== wire round trip ==\n");
  std::string bytes;
  Check(msg.SerializeToString(&bytes), "message serialises");
  aimdk::protocol::TaWholeBodyCommandChannel decoded;
  Check(decoded.ParseFromString(bytes), "message parses back");
  Check(decoded.data().leg_command().angles_rad(3) == msg.data().leg_command().angles_rad(3),
        "leg entry survives serialisation");

  // Mirror of ConvertTaWholeBodyCommand (a3_teleop_reference.cpp): the runtime
  // reads the groups back into the policy view.
  std::printf("== mirror of the official converter ==\n");
  double q_policy[29] = {0.0};
  double dq_policy[29] = {0.0};
  const auto& data = decoded.data();
  for (int i = 0; i < 3; ++i) q_policy[i] = data.waist_command().angles_rad(i);
  for (int i = 0; i < 14; ++i) q_policy[3 + i] = data.arm_command().angles_rad(i);
  for (int i = 0; i < 12; ++i) q_policy[17 + i] = data.leg_command().angles_rad(i);
  // The runtime picks the velocity offsets from the array length, so the mirror
  // must do the same (our message uses the 31-slot layout: arm starts at 17).
  MirrorOfficialVelocityLayout(data.joint_velocities().velocities_rad_s(), dq_policy);

  bool positions_ok = true;
  for (int il = 0; il < 29; ++il) {
    const int policy = kA3IlToPolicyIndex[il];
    if (std::fabs(q_policy[policy] - window.joint_pos_rad[il]) > 1e-9) positions_ok = false;
  }
  Check(positions_ok, "every wire joint index arrives at the policy index the runtime reads");

  bool velocities_ok = true;
  for (int il = 0; il < 29; ++il) {
    const int policy = kA3IlToPolicyIndex[il];
    if (std::fabs(dq_policy[policy] - window.joint_vel_rad_s[il]) > 1e-9) velocities_ok = false;
  }
  Check(velocities_ok, "every wire velocity arrives at the right policy index");

  std::printf("== head variant ==\n");
  A3WholeBodyCommandFields with_head = fields;
  with_head.has_head_command = true;
  with_head.head_angles_rad = {0.3, -0.2};
  aimdk::protocol::TaWholeBodyCommandChannel head_msg;
  FillWholeBodyCommandChannel(with_head, &head_msg, 1, false);
  Check(head_msg.data().has_head_command(), "head command present when requested");
  Check(head_msg.data().joint_velocities().velocities_rad_s_size() == 30,
        "the 30-slot velocity variant is available too");

  std::printf("\n%s (%d failure(s))\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
  return g_failures == 0 ? 0 : 1;
}
