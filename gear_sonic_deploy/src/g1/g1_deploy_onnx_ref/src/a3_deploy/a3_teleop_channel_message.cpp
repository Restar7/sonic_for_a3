// See include/a3_deploy/a3_teleop_channel_message.hpp.

#include "a3_deploy/a3_teleop_channel_message.hpp"

#ifdef HAS_A3_TA_PROTO
#include "aimdk/protocol/ta/ta_channel.pb.h"
#include "aimdk/protocol/ta/ta_whole_body_command.pb.h"

namespace a3_deploy {

void FillWholeBodyCommandChannel(const A3WholeBodyCommandFields& fields,
                                 aimdk::protocol::TaWholeBodyCommandChannel* msg,
                                 std::uint32_t seq,
                                 bool include_head_in_velocity_layout) {
  if (msg == nullptr) return;
  msg->Clear();

  auto* header = msg->mutable_header();
  header->set_seq(seq);
  auto* stamp = header->mutable_timestamp();
  stamp->set_seconds(fields.stamp_ns / 1'000'000'000LL);
  stamp->set_nanos(static_cast<std::int32_t>(fields.stamp_ns % 1'000'000'000LL));

  auto* data = msg->mutable_data();
  // BODY_31 = leg(12) + waist(3) + head(2) + arm(14): the layout the A3 runtime's
  // ConvertTaWholeBodyCommand reads back.
  data->set_joint_layout(aimdk::protocol::TaJointLayout_BODY_31);

  auto* pelvis = data->mutable_pelvis_pose();
  for (double v : fields.pelvis_quat_wxyz) pelvis->add_quat_wxyz(v);
  for (double v : fields.pelvis_position_m) pelvis->add_position_xyz(v);

  auto* leg = data->mutable_leg_command();
  for (double v : fields.leg_angles_rad) leg->add_angles_rad(v);

  auto* waist = data->mutable_waist_command();
  for (double v : fields.waist_angles_rad) waist->add_angles_rad(v);

  auto* arm = data->mutable_arm_command();
  for (double v : fields.arm_angles_rad) arm->add_angles_rad(v);

  // Velocity layout: leg(12) + waist(3) + head(2) + arm(14) = 31 as documented by
  // the proto.  (ConvertTaWholeBodyCommand also accepts a 30-slot variant with a
  // single head entry; we emit the documented 31.)
  auto* velocities = data->mutable_joint_velocities();
  for (int i = 0; i < 12; ++i) velocities->add_velocities_rad_s(fields.velocities_rad_s[i]);
  for (int i = 0; i < 3; ++i) velocities->add_velocities_rad_s(fields.velocities_rad_s[12 + i]);
  if (include_head_in_velocity_layout) {
    velocities->add_velocities_rad_s(0.0);  // head yaw: no head data on the wire
    velocities->add_velocities_rad_s(0.0);  // head pitch
  } else {
    velocities->add_velocities_rad_s(fields.velocities_rad_s[15]);
  }
  for (int i = 0; i < 14; ++i) velocities->add_velocities_rad_s(fields.velocities_rad_s[16 + i]);

  if (fields.has_head_command) {
    auto* head = data->mutable_head_command();
    for (double v : fields.head_angles_rad) head->add_angles_rad(v);
  }
}

}  // namespace a3_deploy
#endif  // HAS_A3_TA_PROTO
