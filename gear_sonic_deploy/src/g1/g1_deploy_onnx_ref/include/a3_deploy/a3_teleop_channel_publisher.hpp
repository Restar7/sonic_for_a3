// Publishes A3WholeBodyCommandFields on /ta/whole_body_command (plan sections 59/60).
//
// This is the only piece of the A3-side teleop path that needs AimRT: everything
// else (decoding, permutation, validation, pumping) lives in
// `a3_teleop_command_source.{hpp,cpp}` and is unit-tested without it.
//
// The header stays AimRT-free on purpose: `Start()` takes the module handle as an
// opaque `void*` (really an `aimrt::CoreRef*`).  When the deploy package is built
// without TA proto support (`HAS_A3_TA_PROTO` undefined -- the default x86 build)
// the implementation compiles to a stub that reports the reason instead of
// failing to build, so the rest of the runtime keeps working unchanged.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "a3_deploy/a3_teleop_command_source.hpp"

namespace a3_deploy {

struct A3TeleopChannelPublisherOptions {
  std::string topic{"/ta/whole_body_command"};
  // Filled into `joint_velocities` as zeros so the 31-slot layout documented by
  // the proto is honoured (leg 12 + waist 3 + head 2 + arm 14).
  bool include_head_in_velocity_layout = true;
};

class A3TeleopChannelPublisher {
 public:
  A3TeleopChannelPublisher();
  ~A3TeleopChannelPublisher();

  A3TeleopChannelPublisher(const A3TeleopChannelPublisher&) = delete;
  A3TeleopChannelPublisher& operator=(const A3TeleopChannelPublisher&) = delete;

  // `module_handle` must point to an `aimrt::CoreRef` (pass &core_ref).
  bool Start(const void* module_handle,
             const A3TeleopChannelPublisherOptions& options,
             std::string* error = nullptr);
  void Stop();
  bool Ready() const;

  // Convert the field groups into the channel message and publish it.
  bool Publish(const A3WholeBodyCommandFields& fields, std::string* error = nullptr);

  std::uint64_t Published() const;
  std::uint64_t PublishFailures() const;

  // "aimrt" when built with HAS_A3_TA_PROTO, "disabled" otherwise.
  static const char* BackendName();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace a3_deploy
