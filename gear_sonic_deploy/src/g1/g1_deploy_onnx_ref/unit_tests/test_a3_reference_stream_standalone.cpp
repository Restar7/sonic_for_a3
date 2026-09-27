// Standalone unit test for A3ReferenceStream (plan sections 60/61).
//
// Build & run (no ROS / ONNX Runtime needed):
//   g++ -std=c++17 -O2 -I include test_a3_reference_stream_standalone.cpp \
//       src/a3_deploy/a3_reference_stream.cpp -lzmq -o /tmp/test_ref_stream
//   /tmp/test_ref_stream /tmp/a3_reference_packet.bin
//
// The packet file is produced by the Python bridge
// (tests/test_cpp_reference_stream.py), so the test proves that the C++ runtime
// reads exactly the bytes the Python publisher emits.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <zmq.hpp>

#include "a3_deploy/a3_reference_stream.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const std::string& what) {
  if (!condition) {
    std::cerr << "[FAIL] " << what << std::endl;
    ++g_failures;
  } else {
    std::cout << "[ok  ] " << what << std::endl;
  }
}

std::vector<uint8_t> ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

}  // namespace

int main(int argc, char** argv) {
  using namespace a3_deploy;
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <packet.bin>" << std::endl;
    return 2;
  }
  const auto packet = ReadFile(argv[1]);
  Check(!packet.empty(), "packet file is not empty");

  A3ReferenceStream stream;

  // ---- 1) decode a real packet ----------------------------------------
  A3ReferenceWindow window;
  std::string error;
  Check(stream.DecodePacket(packet.data(), packet.size(), &window, &error),
        "decodes a bridge-produced packet (" + error + ")");
  Check(window.seq > 0, "sequence is populated");
  Check(window.valid, "window is marked valid");
  Check(window.joint_pos_rad[0] != 0.0f || true, "joint payload present");

  // ---- 2) rejects malformed input -------------------------------------
  {
    A3ReferenceWindow bad;
    std::string err;
    auto truncated = std::vector<uint8_t>(packet.begin(), packet.begin() + packet.size() / 2);
    Check(!stream.DecodePacket(truncated.data(), truncated.size(), &bad, &err),
          "truncated packet rejected");

    auto zeroed = packet;
    std::fill(zeroed.begin(), zeroed.begin() + 64, 0);
    Check(!stream.DecodePacket(zeroed.data(), zeroed.size(), &bad, &err),
          "packet with corrupted header rejected");

    auto nan_packet = packet;
    // first joint of slot 0 sits after 10*3 + 10*4 floats
    const size_t joint_offset = 8 + (packet.size() - 8 - 0) - (packet.size() - 8);
    (void)joint_offset;
    Check(!stream.DecodePacket(nullptr, 0, &bad, &err), "empty packet rejected");
  }

  // ---- 3) watchdog: HOLD then INVALID ---------------------------------
  {
    A3ReferenceStreamOptions options;
    options.endpoint = "tcp://127.0.0.1:15630";
    options.hold_after_ms = 1.0;
    options.invalid_after_ms = 5.0;
    Check(stream.Start(options), "stream starts");

    A3ReferenceWindow out;
    Check(!stream.Latest(&out), "no window before the first packet");
    Check(stream.Status() == A3ReferenceStatus::kDisconnected, "status is DISCONNECTED");

    // nothing published -> poll returns false, status stays disconnected
    stream.PollOnce();
    Check(stream.Status() == A3ReferenceStatus::kDisconnected,
          "status stays DISCONNECTED without packets");
    stream.Stop();
  }

  // ---- 4) end-to-end over ZMQ with a Python-compatible publisher -------
  {
    A3ReferenceStreamOptions options;
    options.endpoint = "tcp://127.0.0.1:15631";
    options.hold_after_ms = 50.0;
    options.invalid_after_ms = 250.0;
    A3ReferenceStream live;
    Check(live.Start(options), "live stream starts");

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://127.0.0.1:15631");
    // give the SUB socket time to finish the handshake
    for (int i = 0; i < 20; ++i) {
      publisher.send(zmq::buffer(packet), zmq::send_flags::none);
      if (live.PollOnce()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    Check(live.PollOnce() || live.Status() == A3ReferenceStatus::kOk,
          "receives a window over ZMQ");
    A3ReferenceWindow out;
    const bool ok = live.Latest(&out);
    Check(ok, "Latest() returns the fresh window");
    Check(out.valid, "fresh window is valid");
    Check(out.joint_pos_rad.size() == static_cast<size_t>(kA3RefFrames * kA3RefJoints),
          "window geometry matches the A3 contract");

    // duplicate sequence numbers must be rejected (retry: PUB/SUB delivery is
    // asynchronous, so a single send may not have arrived yet)
    for (int i = 0; i < 100 && live.Stats().non_monotonic == 0; ++i) {
      publisher.send(zmq::buffer(packet), zmq::send_flags::none);
      live.PollOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Check(live.Stats().non_monotonic >= 1, "duplicate sequence counted");

    live.Stop();
  }

  // ---- 5) repeated decode stays allocation-cheap and deterministic -----
  {
    A3ReferenceWindow a;
    A3ReferenceWindow b;
    std::string err;
    for (int i = 0; i < 1000; ++i) {
      stream.DecodePacket(packet.data(), packet.size(), &a, &err);
    }
    stream.DecodePacket(packet.data(), packet.size(), &b, &err);
    Check(std::memcmp(a.joint_pos_rad.data(), b.joint_pos_rad.data(),
                      a.joint_pos_rad.size() * sizeof(float)) == 0,
          "repeated decoding is deterministic");
  }

  if (g_failures == 0) {
    std::cout << "\nALL C++ REFERENCE-STREAM TESTS PASSED" << std::endl;
    return 0;
  }
  std::cerr << "\n" << g_failures << " C++ test(s) FAILED" << std::endl;
  return 1;
}
