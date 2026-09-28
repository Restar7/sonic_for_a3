// Fills the TA channel message from the bridge field groups (plan sections 59/60).
//
// The wire type is `aimdk::protocol::TaWholeBodyCommandChannel`:
//
//     message TaWholeBodyCommandChannel {
//       Header header = 1;            // seq + timestamp
//       TaWholeBodyCommand data = 2;  // the joint groups
//     }
//
// Keeping this separate from the publisher means the message layout can be tested
// against the real generated protobuf code without AimRT.
#pragma once

#include <cstdint>

#include "a3_deploy/a3_teleop_command_source.hpp"

#ifdef HAS_A3_TA_PROTO
namespace aimdk {
namespace protocol {
class TaWholeBodyCommandChannel;
}  // namespace protocol
}  // namespace aimdk

namespace a3_deploy {

// `seq` goes into header.seq; header.timestamp is set from fields.stamp_ns.
// Head angles are only written when fields.has_head_command is set.
void FillWholeBodyCommandChannel(const A3WholeBodyCommandFields& fields,
                                 aimdk::protocol::TaWholeBodyCommandChannel* msg,
                                 std::uint32_t seq,
                                 bool include_head_in_velocity_layout = true);

}  // namespace a3_deploy
#endif  // HAS_A3_TA_PROTO
