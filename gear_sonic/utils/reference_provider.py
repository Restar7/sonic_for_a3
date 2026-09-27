"""A3 reference providers for SONIC sim2sim / deploy (plan section 39).

The 035 A3-fast encoder consumes a 10-slot future reference window (20 ms
spacing, 0 .. 180 ms).  Historically that window was sliced out of a flat CSV
motion file.  This module introduces a small abstraction so the very same
observation builder can be fed either from the CSV (unchanged behaviour) or from
a live ``A3_REFERENCE_V1`` stream:

    ReferenceProvider (Protocol)
      - reset()
      - get_window(ref_frame, frame_skip, history_frames,
                   valid_future_frames, zero_pad_invalid_frames, on_end)

    CsvReferenceProvider   -> provided by sim2sim_a3_mujoco (wraps MotionReference
                              and reuses its index helpers, so the CSV path stays
                              bit-identical)
    StreamingReferenceProvider -> ZMQ SUB of A3_REFERENCE_V1 windows

The wire format is documented in the bridge repository
(``a3_teleop_bridge/transport/protocol.py``): a single ZMQ frame

    b"A3R1" | uint32 header_len | msgpack header | float32 payload

with the payload concatenating ``root_pos_m[10,3] | root_quat_wxyz[10,4] |
joint_pos_rad[10,29] | joint_vel_rad_s[10,29]``.  A single frame (not multipart)
is used so ``ZMQ_CONFLATE`` keeps working.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import msgpack
import numpy as np

PROTOCOL_VERSION = "A3_REFERENCE_V1"
MAGIC = b"A3R1"
_MAGIC_STRUCT = struct.Struct("<4sI")
ARRAY_ORDER = ("root_pos_m", "root_quat_wxyz", "joint_pos_rad", "joint_vel_rad_s")
DTYPE_MAP = {"<f4": np.dtype("<f4"), "f4": np.dtype("<f4")}


@dataclass
class ReferenceWindow:
    """The encoder-facing slice of a reference stream."""

    anchor_quat_wxyz: np.ndarray  # [F, 4] w, x, y, z
    dof_il: np.ndarray  # [F, 29] policy-order joint positions
    dof_vel_il: np.ndarray  # [F, 29] policy-order joint velocities
    valid: bool = True
    source_age_ms: float = 0.0
    seq: int = 0
    timestamp_ns: int = 0
    state: str = "TRACKING"
    root_pos_m: np.ndarray | None = None
    note: str = ""


@runtime_checkable
class ReferenceProvider(Protocol):
    """Anything that can supply an encoder reference window."""

    def reset(self) -> None:  # pragma: no cover - protocol
        ...

    def get_window(
        self,
        ref_frame: int,
        frame_skip: int,
        history_frames: int = 0,
        valid_future_frames: int | None = None,
        zero_pad_invalid_frames: bool = False,
        on_end: str = "hold_last",
    ) -> ReferenceWindow:  # pragma: no cover - protocol
        ...


def decode_reference_packet(packet: bytes) -> ReferenceWindow:
    """Decode one ``A3_REFERENCE_V1`` frame into a :class:`ReferenceWindow`."""
    if len(packet) < _MAGIC_STRUCT.size:
        raise ValueError(f"packet too short: {len(packet)} bytes")
    magic, header_len = _MAGIC_STRUCT.unpack_from(packet, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r}, expected {MAGIC!r}")
    start = _MAGIC_STRUCT.size
    header_bytes = packet[start : start + header_len]
    payload_bytes = packet[start + header_len :]
    header = msgpack.unpackb(header_bytes, raw=False)
    if header.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"protocol version mismatch: {header.get('version')!r}")
    if header.get("endian", "le") != "le":
        raise ValueError("only little-endian payloads are supported")
    shapes = header["shapes"]
    dtype = np.dtype(header.get("dtype", "<f4"))
    payload = np.frombuffer(payload_bytes, dtype=dtype)
    expected = sum(int(np.prod(shapes[name])) for name in ARRAY_ORDER)
    if payload.size != expected:
        raise ValueError(f"payload holds {payload.size} floats, expected {expected}")

    arrays = {}
    offset = 0
    for name in ARRAY_ORDER:
        shape = tuple(int(v) for v in shapes[name])
        count = int(np.prod(shape))
        arrays[name] = np.array(payload[offset : offset + count], dtype=np.float64).reshape(shape)
        offset += count

    for name in ARRAY_ORDER:
        if not np.isfinite(arrays[name]).all():
            raise ValueError(f"{name} contains non-finite values")

    return ReferenceWindow(
        anchor_quat_wxyz=arrays["root_quat_wxyz"],
        dof_il=arrays["joint_pos_rad"],
        dof_vel_il=arrays["joint_vel_rad_s"],
        valid=bool(header.get("valid", True)),
        source_age_ms=float(header.get("source_age_ms", 0.0)),
        seq=int(header.get("seq", 0)),
        timestamp_ns=int(header.get("timestamp_ns", 0)),
        state=str(header.get("state", "TRACKING")),
        root_pos_m=arrays["root_pos_m"],
    )


@dataclass
class StreamingStats:
    received: int = 0
    rejected: int = 0
    dropped: int = 0
    last_seq: int | None = None
    last_reason: str = ""
    latencies_ms: list[float] = field(default_factory=list)

    def as_dict(self) -> dict:
        lat = np.asarray(self.latencies_ms[-2000:], dtype=np.float64)
        return {
            "received": self.received,
            "rejected": self.rejected,
            "dropped": self.dropped,
            "last_seq": self.last_seq,
            "last_reason": self.last_reason,
            "latency_p50_ms": float(np.percentile(lat, 50)) if lat.size else None,
            "latency_p95_ms": float(np.percentile(lat, 95)) if lat.size else None,
        }


class StreamingReferenceProvider:
    """ZMQ SUB provider for ``A3_REFERENCE_V1`` windows.

    Only the newest packet is ever exposed (``ZMQ_CONFLATE`` + explicit drain),
    so a stalled consumer can never be handed a backlog of old motion.
    """

    def __init__(
        self,
        endpoint: str = "tcp://127.0.0.1:5560",
        context=None,
        recv_timeout_ms: int = 50,
        stale_after_ms: float = 250.0,
        verbose: bool = False,
    ) -> None:
        import zmq  # imported lazily so CSV-only runs keep working without pyzmq

        self.zmq = zmq
        self.endpoint = endpoint
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.RCVTIMEO, recv_timeout_ms)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(endpoint)
        self.stale_after_ms = float(stale_after_ms)
        self.verbose = verbose
        self.stats = StreamingStats()
        self.last_window: ReferenceWindow | None = None
        self.last_receive_time: float | None = None
        self.hold_window: ReferenceWindow | None = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.last_window = None
        self.hold_window = None

    def poll(self) -> ReferenceWindow | None:
        """Drain the socket and decode the newest packet (latest-only)."""
        packet = None
        try:
            packet = self.socket.recv(self.zmq.NOBLOCK)
            while True:
                try:
                    packet = self.socket.recv(self.zmq.NOBLOCK)
                except self.zmq.Again:
                    break
        except self.zmq.Again:
            packet = None
        if packet is None:
            return None
        try:
            window = decode_reference_packet(packet)
        except Exception as exc:  # malformed / NaN / wrong version
            self.stats.rejected += 1
            self.stats.last_reason = str(exc)
            return None
        self.stats.received += 1
        if self.stats.last_seq is not None and window.seq > self.stats.last_seq + 1:
            self.stats.dropped += window.seq - self.stats.last_seq - 1
        self.stats.last_seq = window.seq
        self.stats.latencies_ms.append(window.source_age_ms)
        self.last_window = window
        self.last_receive_time = window.timestamp_ns / 1e9 if window.timestamp_ns else None
        return window

    # ------------------------------------------------------------------
    def get_window(
        self,
        ref_frame: int,
        frame_skip: int,
        history_frames: int = 0,
        valid_future_frames: int | None = None,
        zero_pad_invalid_frames: bool = False,
        on_end: str = "hold_last",
    ) -> ReferenceWindow:
        """Return the newest complete window (signature matches the CSV path)."""
        del ref_frame, frame_skip, history_frames, valid_future_frames, zero_pad_invalid_frames, on_end
        fresh = self.poll()
        if fresh is not None:
            if fresh.valid:
                self.hold_window = fresh
                return fresh
            # an explicitly invalid frame (HOLD / SAFE_STOP) is forwarded as-is
            return fresh
        if self.hold_window is not None:
            held = ReferenceWindow(
                anchor_quat_wxyz=self.hold_window.anchor_quat_wxyz.copy(),
                dof_il=self.hold_window.dof_il.copy(),
                dof_vel_il=np.zeros_like(self.hold_window.dof_vel_il),
                valid=False,
                source_age_ms=self.hold_window.source_age_ms,
                seq=self.hold_window.seq,
                timestamp_ns=self.hold_window.timestamp_ns,
                state="HOLD",
                root_pos_m=None
                if self.hold_window.root_pos_m is None
                else self.hold_window.root_pos_m.copy(),
                note="no packet available; holding the last valid window",
            )
            return held
        raise RuntimeError(
            "no A3_REFERENCE_V1 packet received yet; is the bridge publisher running?"
        )

    def close(self) -> None:
        try:
            self.socket.close(linger=0)
        except Exception:
            pass
