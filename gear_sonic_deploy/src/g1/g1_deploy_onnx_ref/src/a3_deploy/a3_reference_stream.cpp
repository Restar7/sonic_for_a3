// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3_REFERENCE_V1 receiver + watchdog (plan sections 60/61).
#include "a3_deploy/a3_reference_stream.hpp"

#include <zmq.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <msgpack.hpp>

namespace a3_deploy {
namespace {

constexpr char kMagic[4] = {'A', '3', 'R', '1'};
constexpr const char* kProtocolVersion = "A3_REFERENCE_V1";
constexpr const char* kJointOrderTag = "a3_il_v1";

double NowSeconds() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

uint32_t ReadU32LE(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

bool IsFiniteArray(const float* data, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(data[i])) return false;
  }
  return true;
}

}  // namespace

const A3ReferenceLimits& A3ReferenceJointLimits() { return kA3ReferenceLimits; }

bool A3ReferenceValidateWindow(const A3ReferenceWindow& window, std::string* error) {
  auto fail = [&](const std::string& message) {
    if (error) *error = message;
    return false;
  };
  if (!IsFiniteArray(window.root_pos_m.data(), window.root_pos_m.size()) ||
      !IsFiniteArray(window.root_quat_wxyz.data(), window.root_quat_wxyz.size())) {
    return fail("non-finite root state");
  }
  if (!IsFiniteArray(window.joint_pos_rad.data(), window.joint_pos_rad.size()) ||
      !IsFiniteArray(window.joint_vel_rad_s.data(), window.joint_vel_rad_s.size())) {
    return fail("non-finite joint state");
  }
  const auto& limits = A3ReferenceJointLimits();
  for (int frame = 0; frame < kA3RefFrames; ++frame) {
    for (int joint = 0; joint < kA3RefJoints; ++joint) {
      const float q = window.joint_pos_rad[frame * kA3RefJoints + joint];
      const float dq = window.joint_vel_rad_s[frame * kA3RefJoints + joint];
      if (q < limits.position_lower[joint] - limits.position_tolerance ||
          q > limits.position_upper[joint] + limits.position_tolerance) {
        return fail("joint " + std::string(limits.joint_names[joint]) +
                    " position outside limits");
      }
      if (std::fabs(dq) > limits.velocity[joint] * limits.velocity_scale) {
        return fail("joint " + std::string(limits.joint_names[joint]) +
                    " velocity outside limits");
      }
    }
  }
  for (int frame = 0; frame < kA3RefFrames; ++frame) {
    const float w = window.root_quat_wxyz[frame * 4 + 0];
    const float x = window.root_quat_wxyz[frame * 4 + 1];
    const float y = window.root_quat_wxyz[frame * 4 + 2];
    const float z = window.root_quat_wxyz[frame * 4 + 3];
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (std::fabs(norm - 1.0f) > 1e-2f) return fail("root quaternion is not unit length");
  }
  return true;
}

struct A3ReferenceStream::Impl {
  zmq::context_t context{1};
  zmq::socket_t socket{context, zmq::socket_type::sub};
  bool connected = false;
};

A3ReferenceStream::A3ReferenceStream() : impl_(std::make_unique<Impl>()) {}
A3ReferenceStream::~A3ReferenceStream() { Stop(); }

bool A3ReferenceStream::Start(const A3ReferenceStreamOptions& options) {
  options_ = options;
  try {
    impl_->socket.setsockopt(ZMQ_RCVHWM, 1);
    if (options_.conflate) impl_->socket.setsockopt(ZMQ_CONFLATE, 1);
    impl_->socket.setsockopt(ZMQ_RCVTIMEO, options_.recv_timeout_ms);
    impl_->socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    impl_->socket.connect(options_.endpoint);
  } catch (const std::exception& exc) {
    stats_.last_error = std::string("zmq connect failed: ") + exc.what();
    stats_.status = A3ReferenceStatus::kDisconnected;
    return false;
  }
  impl_->connected = true;
  stats_.status = A3ReferenceStatus::kDisconnected;
  return true;
}

void A3ReferenceStream::Stop() {
  if (!impl_ || !impl_->connected) return;
  try {
    impl_->socket.close();
  } catch (...) {
  }
  impl_->connected = false;
}

double A3ReferenceStream::LastReceiveAgeMs() const {
  if (last_receive_time_s_ <= 0.0) return -1.0;
  return (NowSeconds() - last_receive_time_s_) * 1000.0;
}

bool A3ReferenceStream::DecodePacket(const uint8_t* data, size_t size,
                                     A3ReferenceWindow* out, std::string* error) {
  auto fail = [&](const std::string& message) {
    if (error) *error = message;
    return false;
  };
  if (size < 8) return fail("packet too short");
  if (std::memcmp(data, kMagic, 4) != 0) return fail("bad magic");
  const uint32_t header_len = ReadU32LE(data + 4);
  if (size < 8u + header_len) return fail("header length exceeds packet size");

  msgpack::object_handle handle;
  try {
    handle = msgpack::unpack(reinterpret_cast<const char*>(data + 8), header_len);
  } catch (const std::exception& exc) {
    return fail(std::string("cannot unpack header: ") + exc.what());
  }
  const msgpack::object& header = handle.get();
  if (header.type != msgpack::type::MAP) return fail("header is not a map");

  std::string version;
  std::string dtype;
  std::string joint_order;
  bool valid = true;
  std::map<std::string, std::vector<int>> shapes;
  for (uint32_t i = 0; i < header.via.map.size; ++i) {
    const auto& kv = header.via.map.ptr[i];
    const std::string key = kv.key.as<std::string>();
    if (key == "version") {
      version = kv.val.as<std::string>();
    } else if (key == "dtype") {
      dtype = kv.val.as<std::string>();
    } else if (key == "joint_order") {
      joint_order = kv.val.as<std::string>();
    } else if (key == "valid") {
      valid = kv.val.as<bool>();
    } else if (key == "shapes") {
      kv.val.convert(shapes);
    } else if (key == "seq") {
      out->seq = kv.val.as<uint64_t>();
    } else if (key == "timestamp_ns") {
      out->timestamp_ns = kv.val.as<int64_t>();
    } else if (key == "dt") {
      out->dt = kv.val.as<float>();
    } else if (key == "source_age_ms") {
      out->source_age_ms = kv.val.as<float>();
    } else if (key == "solver_latency_ms") {
      out->solver_latency_ms = kv.val.as<float>();
    }
  }

  if (version != kProtocolVersion) {
    return fail("protocol version mismatch: " + version);
  }
  if (joint_order != kJointOrderTag) {
    return fail("joint order mismatch: " + joint_order + " (need " + kJointOrderTag + ")");
  }
  if (dtype != "<f4") return fail("unsupported dtype: " + dtype);

  auto shape_of = [&](const std::string& name, int rows, int cols) {
    auto it = shapes.find(name);
    if (it == shapes.end()) return false;
    return it->second.size() == 2 && it->second[0] == rows && it->second[1] == cols;
  };
  if (!shape_of("root_pos_m", kA3RefFrames, 3) ||
      !shape_of("root_quat_wxyz", kA3RefFrames, 4) ||
      !shape_of("joint_pos_rad", kA3RefFrames, kA3RefJoints) ||
      !shape_of("joint_vel_rad_s", kA3RefFrames, kA3RefJoints)) {
    return fail("payload layout does not match the A3 contract");
  }

  const size_t expected_floats =
      kA3RefFrames * 3 + kA3RefFrames * 4 + 2 * kA3RefFrames * kA3RefJoints;
  const size_t payload_bytes = size - 8u - header_len;
  if (payload_bytes != expected_floats * sizeof(float)) {
    return fail("payload size mismatch");
  }

  const float* payload = reinterpret_cast<const float*>(data + 8 + header_len);
  size_t offset = 0;
  std::memcpy(out->root_pos_m.data(), payload + offset, out->root_pos_m.size() * sizeof(float));
  offset += out->root_pos_m.size();
  std::memcpy(out->root_quat_wxyz.data(), payload + offset,
              out->root_quat_wxyz.size() * sizeof(float));
  offset += out->root_quat_wxyz.size();
  std::memcpy(out->joint_pos_rad.data(), payload + offset,
              out->joint_pos_rad.size() * sizeof(float));
  offset += out->joint_pos_rad.size();
  std::memcpy(out->joint_vel_rad_s.data(), payload + offset,
              out->joint_vel_rad_s.size() * sizeof(float));

  out->valid = valid;

  if (!A3ReferenceValidateWindow(*out, error)) {
    return false;
  }
  return true;
}

bool A3ReferenceStream::PollOnce() {
  if (!impl_ || !impl_->connected) return false;

  std::vector<uint8_t> packet;
  try {
    zmq::message_t message;
    bool got = false;
    // latest-only: keep draining so a stalled consumer never sees a backlog
    while (impl_->socket.recv(message, zmq::recv_flags::dontwait)) {
      packet.assign(static_cast<uint8_t*>(message.data()),
                    static_cast<uint8_t*>(message.data()) + message.size());
      got = true;
    }
    if (!got) return false;
  } catch (const std::exception& exc) {
    stats_.last_error = std::string("recv failed: ") + exc.what();
    return false;
  }

  A3ReferenceWindow window;
  std::string error;
  if (!DecodePacket(packet.data(), packet.size(), &window, &error)) {
    stats_.rejected += 1;
    stats_.last_error = error;
    if (error.find("non-finite") != std::string::npos ||
        error.find("not unit length") != std::string::npos) {
      stats_.nan_packets += 1;
    } else if (error.find("outside limits") != std::string::npos) {
      stats_.limit_violations += 1;
    }
    return false;
  }

  if (has_latest_ && window.seq <= latest_.seq) {
    stats_.non_monotonic += 1;
    stats_.last_error = "non-monotonic sequence";
    return false;
  }
  if (has_latest_ && window.seq > latest_.seq + 1) {
    stats_.dropped += window.seq - latest_.seq - 1;
  }

  stats_.received += 1;
  last_receive_time_s_ = NowSeconds();
  window.local_age_ms = 0.0;
  latest_ = window;
  has_latest_ = true;
  if (window.valid) {
    held_ = window;
    stats_.status = A3ReferenceStatus::kOk;
  } else {
    stats_.invalid_packets += 1;
    stats_.status = A3ReferenceStatus::kHold;
  }
  return true;
}

bool A3ReferenceStream::Latest(A3ReferenceWindow* out) {
  if (!has_latest_) {
    stats_.status = A3ReferenceStatus::kDisconnected;
    return false;
  }
  const double age_ms = LastReceiveAgeMs();
  if (age_ms > options_.invalid_after_ms) {
    stats_.status = A3ReferenceStatus::kInvalid;
  } else if (age_ms > options_.hold_after_ms) {
    stats_.status = A3ReferenceStatus::kHold;
  } else if (stats_.status != A3ReferenceStatus::kOk) {
    // keep the last classification until fresh data arrives
  } else {
    stats_.status = A3ReferenceStatus::kOk;
  }

  if (stats_.status == A3ReferenceStatus::kOk) {
    *out = latest_;
    out->local_age_ms = age_ms;
    return out->valid;
  }
  // HOLD / INVALID: freeze the last safe window, zero its velocities, and mark
  // it invalid so downstream code can go to a safe state.  Never extrapolate.
  *out = held_;
  out->joint_vel_rad_s.fill(0.0f);
  out->valid = false;
  out->local_age_ms = age_ms;
  return false;
}

}  // namespace a3_deploy
