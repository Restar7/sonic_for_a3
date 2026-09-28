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
import time

import numpy as np

PROTOCOL_VERSION = "A3_REFERENCE_V1"
#: ``joint_pos_rad`` / ``joint_vel_rad_s`` are published in the order this encoder
#: consumes (URDF/IsaacLab ``dof_il`` order).  The CSV/MJCF policy order is a
#: different permutation of the same joints -- mixing them up silently destroys
#: the policy input.
JOINT_ORDER_TAG = "a3_il_v1"
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
    joint_order = header.get("joint_order")
    if joint_order != JOINT_ORDER_TAG:
        raise ValueError(
            f"joint order mismatch: packet says {joint_order!r}, this encoder needs "
            f"{JOINT_ORDER_TAG!r}"
        )
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
    jumps: int = 0
    max_jump_ms: float = 0.0
    interpolated: int = 0
    max_gap_ms: float = 0.0
    last_seq: int | None = None
    last_reason: str = ""
    latencies_ms: list[float] = field(default_factory=list)

    def as_dict(self) -> dict:
        lat = np.asarray(self.latencies_ms[-2000:], dtype=np.float64)
        return {
            "received": self.received,
            "rejected": self.rejected,
            "dropped": self.dropped,
            "jumps": self.jumps,
            "max_jump_ms": self.max_jump_ms,
            "interpolated": self.interpolated,
            "max_gap_ms": self.max_gap_ms,
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
        slot_ms: float = 20.0,
        startup_window: "ReferenceWindow | None" = None,
        startup_wait_s: float = 30.0,
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
        # Standing reference used until the first real window arrives; None keeps
        # the old behaviour (abort immediately when nothing was ever received).
        self.startup_window = startup_window
        self.startup_wait_s = float(startup_wait_s)
        self._created_monotonic = time.monotonic()
        self._startup_warned = False
        self.verbose = verbose
        #: encoder reference slot spacing (A3-fast contract: 10 slots x 20 ms)
        self.slot_ms = float(slot_ms)
        self.stats = StreamingStats()
        self.last_window: ReferenceWindow | None = None
        self.last_receive_time: float | None = None
        self.hold_window: ReferenceWindow | None = None
        # Window handed to the encoder, plus the newest one received.  They differ
        # while the consumer catches up (see get_window).
        self._current: ReferenceWindow | None = None
        self._pending: ReferenceWindow | None = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.last_window = None
        self.hold_window = None
        self._current = None
        self._pending = None

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
        if self.stats.last_seq is not None and window.seq > self.stats.last_seq + 1:
            self.stats.dropped += window.seq - self.stats.last_seq - 1
        self.stats.last_seq = window.seq
        self.stats.latencies_ms.append(window.source_age_ms)
        # local age of the window relative to the previous one we handed out; a
        # large value means the consumer stalled and the reference jumped
        if self.last_window is not None:
            delta_ms = (window.timestamp_ns - self.last_window.timestamp_ns) / 1e6
            if delta_ms > 60.0:
                self.stats.jumps += 1
                self.stats.max_jump_ms = max(self.stats.max_jump_ms, delta_ms)
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
        if fresh is not None and fresh.valid:
            self._pending = fresh
            self.hold_window = fresh
        elif fresh is not None and not fresh.valid:
            # an explicitly invalid frame (HOLD / SAFE_STOP) is forwarded as-is
            self._pending = None
            self._current = None
            return fresh

        if self._pending is None:
            if self.hold_window is None:
                # Startup: the bridge needs ~15 s to assemble its online UMR session,
                # so hold a standing reference for a bounded time instead of aborting
                # the policy loop (the A3 runtime does the same: it feeds a default
                # standing prefix while TELEOP has no usable window yet).
                elapsed_s = time.monotonic() - self._created_monotonic
                if self.startup_window is not None and elapsed_s < self.startup_wait_s:
                    if not self._startup_warned:
                        self._startup_warned = True
                        print(
                            "[reference-stream] no A3_REFERENCE_V1 packet yet; holding the "
                            f"startup pose for up to {self.startup_wait_s:.0f} s "
                            "(is the PICO sender RUNNING and the bridge publishing?)",
                            flush=True,
                        )
                    return self._startup_hold()
                raise RuntimeError(
                    "no A3_REFERENCE_V1 packet received yet; is the bridge publisher running?"
                )
            return self._blend_hold()

        if self._current is None:
            self._current = self._pending
            return self._current

        gap_ms = (self._pending.timestamp_ns - self._current.timestamp_ns) / 1e6
        slot_ms = self.slot_ms
        if gap_ms <= 0.0:
            # the publisher restarted or the clocks skewed: adopt the newest window
            self._current = self._pending
            return self._current
        alpha = 1.0 if gap_ms <= slot_ms else min(1.0, slot_ms / gap_ms)
        if alpha < 1.0:
            self.stats.interpolated += 1
            self.stats.max_gap_ms = max(self.stats.max_gap_ms, gap_ms)
        self._current = self._blend(self._current, self._pending, alpha)
        return self._current

    @staticmethod
    def _blend(a: ReferenceWindow, b: ReferenceWindow, alpha: float) -> ReferenceWindow:
        """Blend two windows; alpha=1 reproduces ``b`` exactly.

        Nlerp with a hemisphere fix is used for the root quaternion: the gaps
        smoothed here are a few 20 ms slots, far below the range where slerp and
        nlerp differ meaningfully, and it keeps the per-tick cost flat.
        """
        if alpha >= 1.0:
            return b
        joint_pos = (1.0 - alpha) * a.dof_il + alpha * b.dof_il
        joint_vel = (1.0 - alpha) * a.dof_vel_il + alpha * b.dof_vel_il
        quat_a = a.anchor_quat_wxyz
        quat_b = b.anchor_quat_wxyz.copy()
        flip = np.sum(quat_a * quat_b, axis=1) < 0.0
        quat_b[flip] *= -1.0
        quat = (1.0 - alpha) * quat_a + alpha * quat_b
        norms = np.linalg.norm(quat, axis=1, keepdims=True)
        quat = quat / np.where(norms > 1e-9, norms, 1.0)
        root_pos = None
        if a.root_pos_m is not None and b.root_pos_m is not None:
            root_pos = (1.0 - alpha) * a.root_pos_m + alpha * b.root_pos_m
        return ReferenceWindow(
            anchor_quat_wxyz=quat,
            dof_il=joint_pos,
            dof_vel_il=joint_vel,
            valid=b.valid,
            source_age_ms=b.source_age_ms,
            seq=b.seq,
            timestamp_ns=int(round((1.0 - alpha) * a.timestamp_ns + alpha * b.timestamp_ns)),
            state=b.state,
            root_pos_m=root_pos,
            note="interpolated",
        )

    def _startup_hold(self) -> ReferenceWindow:
        """Standing placeholder used until the first real window arrives."""
        base = self.startup_window
        return ReferenceWindow(
            anchor_quat_wxyz=base.anchor_quat_wxyz.copy(),
            dof_il=base.dof_il.copy(),
            dof_vel_il=np.zeros_like(base.dof_vel_il),
            valid=True,
            source_age_ms=0.0,
            seq=0,
            timestamp_ns=0,
            state="CALIBRATION",
            root_pos_m=None if base.root_pos_m is None else base.root_pos_m.copy(),
            note="startup_stand",
        )

    def _blend_hold(self) -> ReferenceWindow:
        """No packet available: freeze the last valid window, velocities zeroed."""
        base = self.hold_window
        frozen = ReferenceWindow(
            anchor_quat_wxyz=base.anchor_quat_wxyz.copy(),
            dof_il=base.dof_il.copy(),
            dof_vel_il=np.zeros_like(base.dof_vel_il),
            valid=False,
            source_age_ms=base.source_age_ms,
            seq=base.seq,
            timestamp_ns=base.timestamp_ns,
            state="HOLD",
            root_pos_m=None if base.root_pos_m is None else base.root_pos_m.copy(),
            note="no packet available; holding the last valid window",
        )
        self._current = None
        return frozen

    def close(self) -> None:
        try:
            self.socket.close(linger=0)
        except Exception:
            pass

    def stats_dict(self) -> dict:
        return self.stats.as_dict()
