// See include/a3_deploy/a3_teleop_channel_publisher.hpp.

#include "a3_deploy/a3_teleop_channel_publisher.hpp"

#include <array>
#include <cstring>
#include <utility>

#include "a3_deploy/a3_teleop_channel_message.hpp"

#ifdef HAS_A3_TA_PROTO
#include <aimrt_module_cpp_interface/core.h>

#include <aimrt_module_protobuf_interface/channel/protobuf_channel.h>

#include "aimdk/protocol/ta/ta_channel.pb.h"
#include "aimdk/protocol/ta/ta_whole_body_command.pb.h"
#endif

namespace a3_deploy {

namespace {

constexpr char kDisabledMessage[] =
    "A3 teleop channel publisher built without HAS_A3_TA_PROTO "
    "(configure the deploy package with the TA proto / AimRT support enabled)";

}  // namespace

struct A3TeleopChannelPublisher::Impl {
  A3TeleopChannelPublisherOptions options{};
  bool ready = false;
  std::uint64_t published = 0;
  std::uint64_t failures = 0;
#ifdef HAS_A3_TA_PROTO
  std::shared_ptr<aimrt::channel::PublisherProxy<aimdk::protocol::TaWholeBodyCommandChannel>>
      proxy;
#endif
};

A3TeleopChannelPublisher::A3TeleopChannelPublisher() : impl_(std::make_unique<Impl>()) {}

A3TeleopChannelPublisher::~A3TeleopChannelPublisher() { Stop(); }

const char* A3TeleopChannelPublisher::BackendName() {
#ifdef HAS_A3_TA_PROTO
  return "aimrt";
#else
  return "disabled";
#endif
}

bool A3TeleopChannelPublisher::Start(const void* module_handle,
                                     const A3TeleopChannelPublisherOptions& options,
                                     std::string* error) {
  impl_->options = options;
#ifdef HAS_A3_TA_PROTO
  if (module_handle == nullptr) {
    if (error) *error = "module handle must not be null";
    return false;
  }
  // The same pattern the existing backends use: get the channel handle from the
  // module, register the proto type once, then publish through a proxy.
  const auto* core_ref = static_cast<const aimrt::CoreRef*>(module_handle);
  auto channel = core_ref->GetChannelHandle();
  auto publisher = channel.GetPublisher(options.topic);
  if (!aimrt::channel::RegisterPublishType<aimdk::protocol::TaWholeBodyCommandChannel>(publisher)) {
    if (error) *error = "RegisterPublishType failed for " + options.topic;
    return false;
  }
  impl_->proxy = std::make_shared<
      aimrt::channel::PublisherProxy<aimdk::protocol::TaWholeBodyCommandChannel>>(publisher);
  impl_->ready = true;
  return true;
#else
  (void)module_handle;
  if (error) *error = kDisabledMessage;
  impl_->ready = false;
  return false;
#endif
}

void A3TeleopChannelPublisher::Stop() {
  impl_->ready = false;
#ifdef HAS_A3_TA_PROTO
  impl_->proxy.reset();
#endif
}

bool A3TeleopChannelPublisher::Ready() const { return impl_->ready; }

bool A3TeleopChannelPublisher::Publish(const A3WholeBodyCommandFields& fields,
                                       std::string* error) {
#ifdef HAS_A3_TA_PROTO
  if (!impl_->ready || impl_->proxy == nullptr) {
    if (error) *error = "publisher is not ready (call Start first)";
    ++impl_->failures;
    return false;
  }
  // The message layout lives in a3_teleop_channel_message.cpp so it can be tested
  // against the real generated protobuf code without AimRT.
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  FillWholeBodyCommandChannel(fields, &msg, static_cast<std::uint32_t>(impl_->published),
                              impl_->options.include_head_in_velocity_layout);

  impl_->proxy->Publish(msg);
  ++impl_->published;
  return true;
#else
  (void)fields;
  if (error) *error = kDisabledMessage;
  ++impl_->failures;
  return false;
#endif
}

std::uint64_t A3TeleopChannelPublisher::Published() const { return impl_->published; }

std::uint64_t A3TeleopChannelPublisher::PublishFailures() const { return impl_->failures; }

}  // namespace a3_deploy
