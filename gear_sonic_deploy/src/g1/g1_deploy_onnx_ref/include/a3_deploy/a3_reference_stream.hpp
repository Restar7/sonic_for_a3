// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3ReferenceStream: A3_REFERENCE_V1 subscriber + watchdog for the A3 runtime.
//
// Plan sections 60/61: after the Python/MuJoCo chain is stable, the SAME wire
// format is consumed by the C++ runtime:
//
//   b"A3R1" | uint32 header_len | msgpack header | float32 payload
//
//   payload = root_pos_m[10,3] | root_quat_wxyz[10,4]
//             | joint_pos_rad[10,29] | joint_vel_rad_s[10,29]
//
// The joint arrays are in the ENCODER order (`dof_il`, i.e. the URDF/IsaacLab
// order that A3ObsBuilder consumes), tagged `joint_order = "a3_il_v1"` in the
// header.  The CSV/MJCF policy order is a different permutation of the same
// joints; see a3_encoder_obs_builder.hpp.
//
// The watchdog mirrors the safety rules of the Python bridge:
//   * reference age > hold_after_ms    -> HOLD  (freeze the last safe window)
//   * reference age > invalid_after_ms -> INVALID (downstream safe halt)
//   * non-monotonic sequence           -> reject the packet
//   * valid == false                   -> propagate HOLD/INVALID
//   * NaN/Inf                          -> reject the packet
//   * joint position / velocity limits -> reject (never silently clamp)
//
// Everything is allocation-free after Start().
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "a3_deploy/a3_reference_limits.hpp"

namespace a3_deploy {

// Window geometry must match the A3-fast encoder contract.
inline constexpr int kA3RefFrames = 10;
inline constexpr int kA3RefJoints = 29;

struct A3ReferenceWindow {
  std::array<float, kA3RefFrames * 3> root_pos_m{};
  std::array<float, kA3RefFrames * 4> root_quat_wxyz{};
  std::array<float, kA3RefFrames * kA3RefJoints> joint_pos_rad{};
  std::array<float, kA3RefFrames * kA3RefJoints> joint_vel_rad_s{};
  uint64_t seq = 0;
  int64_t timestamp_ns = 0;
  float dt = 0.02f;
  float source_age_ms = 0.0f;
  float solver_latency_ms = 0.0f;
  bool valid = false;
  // age of the window measured locally at the moment it was handed out
  mutable double local_age_ms = 0.0;
};

enum class A3ReferenceStatus {
  kDisconnected = 0,  // never received a packet
  kOk = 1,            // fresh and valid
  kHold = 2,          // stale: freeze the last safe window
  kInvalid = 3,       // too stale / invalid: downstream safe halt
};

struct A3ReferenceStats {
  uint64_t received = 0;
  uint64_t rejected = 0;
  uint64_t dropped = 0;
  uint64_t non_monotonic = 0;
  uint64_t invalid_packets = 0;
  uint64_t limit_violations = 0;
  uint64_t nan_packets = 0;
  std::string last_error;
  A3ReferenceStatus status = A3ReferenceStatus::kDisconnected;
};

struct A3ReferenceStreamOptions {
  bool enabled = false;
  std::string endpoint{"tcp://127.0.0.1:5560"};
  double hold_after_ms = 50.0;
  double invalid_after_ms = 250.0;
  int recv_timeout_ms = 50;
  bool conflate = true;
  bool verbose = false;
  // Reject packets whose joints leave the model limits instead of clamping them.
  bool enforce_limits = true;
};

class A3ReferenceStream {
 public:
  A3ReferenceStream();
  ~A3ReferenceStream();

  A3ReferenceStream(const A3ReferenceStream&) = delete;
  A3ReferenceStream& operator=(const A3ReferenceStream&) = delete;

  // Connect the SUB socket.  Returns false (with LastError()) on failure.
  bool Start(const A3ReferenceStreamOptions& options);
  void Stop();

  // Drain the socket (latest-only) and decode the newest packet.
  // Returns true when a new window was accepted.
  bool PollOnce();

  // The window a policy tick should use (last accepted, or the held one).
  // Never extrapolates: on stale input the cached window is returned unchanged
  // with valid=false.
  // Not const: it updates the watchdog status that Stats() reports.
  bool Latest(A3ReferenceWindow* out);

  A3ReferenceStatus Status() const { return stats_.status; }
  const A3ReferenceStats& Stats() const { return stats_; }
  const std::string& LastError() const { return stats_.last_error; }

  double LastReceiveAgeMs() const;

  // Decode one packet without ZMQ (used by unit tests and replay tools).
  bool DecodePacket(const uint8_t* data, size_t size, A3ReferenceWindow* out,
                    std::string* error);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  A3ReferenceStreamOptions options_{};
  A3ReferenceWindow latest_{};
  A3ReferenceWindow held_{};
  A3ReferenceStats stats_{};
  double last_receive_time_s_ = 0.0;
  bool has_latest_ = false;
};

// Exposed for testing / tooling.
bool A3ReferenceValidateWindow(const A3ReferenceWindow& window, std::string* error);
const A3ReferenceLimits& A3ReferenceJointLimits();

}  // namespace a3_deploy
