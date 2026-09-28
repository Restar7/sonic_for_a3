// See include/a3_deploy/a3_teleop_command_source.hpp for the contract this file
// implements.  Nothing here depends on AimRT: the robot-side wiring hands the
// produced fields to the channel and publishes them.

#include "a3_deploy/a3_teleop_command_source.hpp"

#include <cmath>
#include <cstdio>

namespace a3_deploy {
namespace {

// Policy view -> channel groups (see ConvertTaWholeBodyCommand):
//   waist[0..2] = policy[0..2]
//   arm[0..6]   = policy[3..9]     (left arm)
//   arm[7..13]  = policy[10..16]   (right arm)
//   leg[0..5]   = policy[17..22]   (left leg)
//   leg[6..11]  = policy[23..28]   (right leg)
constexpr int kPolicyWaistOffset = 0;
constexpr int kPolicyLeftArmOffset = 3;
constexpr int kPolicyRightArmOffset = 10;
constexpr int kPolicyLeftLegOffset = 17;
constexpr int kPolicyRightLegOffset = 23;

// Lift the channel field groups back into the policy view (the exact inverse of
// the grouping used when the fields were built).
std::array<double, kA3TeleopJointCount> FieldsToPolicyView(
    const A3WholeBodyCommandFields& fields) {
  std::array<double, kA3TeleopJointCount> policy_q{};
  for (int i = 0; i < 3; ++i) policy_q[kPolicyWaistOffset + i] = fields.waist_angles_rad[i];
  for (int i = 0; i < 7; ++i) {
    policy_q[kPolicyLeftArmOffset + i] = fields.arm_angles_rad[i];
    policy_q[kPolicyRightArmOffset + i] = fields.arm_angles_rad[7 + i];
  }
  for (int i = 0; i < 6; ++i) {
    policy_q[kPolicyLeftLegOffset + i] = fields.leg_angles_rad[i];
    policy_q[kPolicyRightLegOffset + i] = fields.leg_angles_rad[6 + i];
  }
  return policy_q;
}

bool CheckFinite(const A3ReferenceWindow& window, std::string* error) {
  for (float v : window.joint_pos_rad) {
    if (!std::isfinite(v)) {
      if (error) *error = "reference window has a non-finite joint position";
      return false;
    }
  }
  for (float v : window.root_quat_wxyz) {
    if (!std::isfinite(v)) {
      if (error) *error = "reference window has a non-finite root quaternion";
      return false;
    }
  }
  for (float v : window.root_pos_m) {
    if (!std::isfinite(v)) {
      if (error) *error = "reference window has a non-finite root position";
      return false;
    }
  }
  return true;
}

// kA3ReferenceLimits / kA3ReferenceJointNames are in ENCODER (il) order, i.e. the
// same index space as the wire payload, so limits are checked before permuting.
bool CheckLimits(const A3ReferenceWindow& window, int frame_index, std::string* error) {
  const int base = frame_index * kA3RefJoints;
  for (int i = 0; i < kA3RefJoints; ++i) {
    const double q = static_cast<double>(window.joint_pos_rad[base + i]);
    const double lo = static_cast<double>(kA3ReferenceLimits.position_lower[i]) -
                      static_cast<double>(kA3ReferenceLimits.position_tolerance);
    const double hi = static_cast<double>(kA3ReferenceLimits.position_upper[i]) +
                      static_cast<double>(kA3ReferenceLimits.position_tolerance);
    if (q < lo || q > hi) {
      if (error) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "joint %s out of range: %.4f not in [%.4f, %.4f]",
                      kA3ReferenceJointNames[i], q, lo, hi);
        *error = buf;
      }
      return false;
    }
  }
  return true;
}

}  // namespace

bool BuildWholeBodyCommandFromReference(const A3ReferenceWindow& window,
                                        int frame_index,
                                        std::int64_t stamp_ns,
                                        A3WholeBodyCommandFields* out,
                                        std::string* error) {
  if (out == nullptr) {
    if (error) *error = "out must not be null";
    return false;
  }
  if (frame_index < 0 || frame_index >= kA3RefFrames) {
    if (error) *error = "frame_index out of range";
    return false;
  }
  if (!window.valid) {
    if (error) *error = "reference window is marked invalid";
    return false;
  }
  if (stamp_ns <= 0) {
    if (error) *error = "stamp_ns must be positive";
    return false;
  }
  if (!CheckFinite(window, error)) return false;

  const int base_q = frame_index * kA3RefJoints;
  const int base_dq = frame_index * kA3RefJoints;

  // wire (il) -> policy view, by the generated permutation only
  std::array<double, kA3TeleopJointCount> policy_q{};
  std::array<double, kA3TeleopJointCount> policy_dq{};
  for (std::size_t il = 0; il < kA3TeleopJointCount; ++il) {
    const int policy = kA3IlToPolicyIndex[il];
    policy_q[policy] = static_cast<double>(window.joint_pos_rad[base_q + il]);
    policy_dq[policy] = static_cast<double>(window.joint_vel_rad_s[base_dq + il]);
  }

  A3WholeBodyCommandFields fields{};
  fields.stamp_ns = stamp_ns;
  for (int i = 0; i < 4; ++i) {
    fields.pelvis_quat_wxyz[i] = static_cast<double>(window.root_quat_wxyz[frame_index * 4 + i]);
  }
  for (int i = 0; i < 3; ++i) {
    fields.pelvis_position_m[i] = static_cast<double>(window.root_pos_m[frame_index * 3 + i]);
  }
  for (int i = 0; i < 3; ++i) {
    fields.waist_angles_rad[i] = policy_q[kPolicyWaistOffset + i];
  }
  for (int i = 0; i < 7; ++i) {
    fields.arm_angles_rad[i] = policy_q[kPolicyLeftArmOffset + i];
    fields.arm_angles_rad[7 + i] = policy_q[kPolicyRightArmOffset + i];
  }
  for (int i = 0; i < 6; ++i) {
    fields.leg_angles_rad[i] = policy_q[kPolicyLeftLegOffset + i];
    fields.leg_angles_rad[6 + i] = policy_q[kPolicyRightLegOffset + i];
  }
  // Protocol velocity layout: leg(12) + waist(3) + head(1) + arm(14).
  for (int i = 0; i < 6; ++i) {
    fields.velocities_rad_s[i] = policy_dq[kPolicyLeftLegOffset + i];
    fields.velocities_rad_s[6 + i] = policy_dq[kPolicyRightLegOffset + i];
  }
  for (int i = 0; i < 3; ++i) {
    fields.velocities_rad_s[12 + i] = policy_dq[kPolicyWaistOffset + i];
  }
  fields.velocities_rad_s[15] = 0.0;  // head: A3_REFERENCE_V1 carries no head data
  for (int i = 0; i < 7; ++i) {
    fields.velocities_rad_s[16 + i] = policy_dq[kPolicyLeftArmOffset + i];
    fields.velocities_rad_s[23 + i] = policy_dq[kPolicyRightArmOffset + i];
  }
  fields.has_velocities = true;
  // No head command: the runtime keeps its own head target while teleoping.
  fields.has_head_command = false;
  fields.head_angles_rad = {0.0, 0.0};

  *out = fields;
  return true;
}

A3TeleopCommandPump::A3TeleopCommandPump(A3TeleopCommandPumpOptions options)
    : options_(options) {}

void A3TeleopCommandPump::Configure(const A3TeleopCommandPumpOptions& options) {
  options_ = options;
  Reset();
}

void A3TeleopCommandPump::Reset() {
  clock_calibrated_ = false;
  clock_offset_ns_ = 0;
  last_push_ns_ = 0;
  last_receive_ns_ = 0;
  last_seq_ = 0;
  last_policy_q_.fill(0.0);
  have_last_q_ = false;
}

bool A3TeleopCommandPump::ShouldHold(std::int64_t now_ns) const {
  if (last_receive_ns_ == 0) return true;
  const double age_ms = static_cast<double>(now_ns - last_receive_ns_) / 1e6;
  return age_ms > options_.stale_after_ms;
}

bool A3TeleopCommandPump::PushWindow(const A3ReferenceWindow& window,
                                     std::int64_t receive_ns,
                                     std::vector<A3WholeBodyCommandFields>* out,
                                     std::string* error) {
  if (out == nullptr) {
    if (error) *error = "out must not be null";
    return false;
  }
  out->clear();

  if (!window.valid) {
    ++stats_.windows_rejected;
    stats_.last_error = "window marked invalid";
    if (error) *error = stats_.last_error;
    return false;
  }
  if (window.timestamp_ns <= 0 || receive_ns <= 0) {
    ++stats_.windows_rejected;
    stats_.last_error = "timestamp must be positive";
    if (error) *error = stats_.last_error;
    return false;
  }
  if (window.seq != 0 && window.seq <= last_seq_ && last_seq_ != 0) {
    // late/duplicate window: the stream is monotonic by contract
    ++stats_.windows_rejected;
    stats_.last_error = "non-monotonic window sequence";
    if (error) *error = stats_.last_error;
    return false;
  }
  if (!CheckFinite(window, error)) {
    ++stats_.windows_rejected;
    ++stats_.rejected_nan;
    stats_.last_error = error ? *error : "non-finite";
    return false;
  }
  if (options_.enforce_limits) {
    for (int f = 0; f < kA3RefFrames; ++f) {
      if (!CheckLimits(window, f, error)) {
        ++stats_.windows_rejected;
        ++stats_.rejected_limits;
        stats_.last_error = error ? *error : "limit violation";
        return false;
      }
    }
  }

  if (!clock_calibrated_) {
    clock_offset_ns_ = receive_ns - window.timestamp_ns;
    clock_calibrated_ = true;
  }

  // Integer step: a float dt accumulates rounding and would make an exactly
  // overlapping window look like a fresh one (duplicate commands).
  const std::int64_t step_ns = std::llround(static_cast<double>(window.dt) * 1e9);
  const int frames = options_.emit_all_window_frames ? kA3RefFrames : 1;
  for (int f = 0; f < frames; ++f) {
    const std::int64_t stamp_ns =
        window.timestamp_ns + static_cast<std::int64_t>(f) * step_ns + clock_offset_ns_;
    if (stamp_ns <= last_push_ns_) {
      ++stats_.frames_skipped_stale_stamp;
      continue;
    }
    A3WholeBodyCommandFields fields{};
    std::string build_error;
    if (!BuildWholeBodyCommandFromReference(window, f, stamp_ns, &fields, &build_error)) {
      ++stats_.windows_rejected;
      ++stats_.rejected_nan;
      if (error) *error = build_error;
      stats_.last_error = build_error;
      return false;
    }
    const std::array<double, kA3TeleopJointCount> policy_q = FieldsToPolicyView(fields);
    if (options_.max_joint_step_rad > 0.0 && have_last_q_) {
      double worst = 0.0;
      for (std::size_t i = 0; i < kA3TeleopJointCount; ++i) {
        worst = std::max(worst, std::fabs(policy_q[i] - last_policy_q_[i]));
      }
      if (worst > options_.max_joint_step_rad) {
        ++stats_.windows_rejected;
        ++stats_.rejected_step;
        stats_.last_error = "reference jumped between consecutive ticks";
        if (error) *error = stats_.last_error;
        return false;
      }
    }
    last_policy_q_ = policy_q;
    have_last_q_ = true;
    last_push_ns_ = stamp_ns;
    out->push_back(fields);
  }

  last_receive_ns_ = receive_ns;
  last_seq_ = window.seq;
  ++stats_.windows_accepted;
  stats_.frames_emitted += out->size();
  return !out->empty();
}

}  // namespace a3_deploy
