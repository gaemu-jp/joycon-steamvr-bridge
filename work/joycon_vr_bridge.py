"""
Joy-Con pose bridge.

This module turns Joy-Con orientation data into a normalized hand pose stream
that can be consumed by a SteamVR-side bridge or another VR runtime adapter.

The companion native OpenVR driver under ``steamvr_driver`` consumes this
bridge's UDP packets and registers the controllers with SteamVR.

The code is intentionally dependency-light so it can run in a fresh Python
environment. It accepts pose samples from:
- stdin as JSON lines
- TCP JSON lines from localhost
- UDP datagrams as JSON
- a built-in simulator for smoke testing

Sample input event:
{"device":"right","quat":[0.0,0.0,0.0,1.0],"buttons":{"a":1},"stick":[0.0,0.0]}

Sample output packet:
{
  "device": "right",
  "timestamp": 1234567890.123,
  "quat": [0.0, 0.0, 0.0, 1.0],
  "euler_deg": [0.0, 0.0, 0.0],
  "buttons": {"a": true},
  "stick": [0.0, 0.0]
}
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import struct
import threading
import socket
import sys
import time
import zlib
from typing import Dict, Iterator, Optional, Tuple


Quat = Tuple[float, float, float, float]
# Measured direct-HID axis mapping: raw X->OpenVR -Z, raw Y->OpenVR -X,
# raw Z->OpenVR +Y. Keep this basis conversion separate from the neutral
# controller-model pose so fixing rotation axes does not disturb hand pose.
JOYCON_AXIS_BASIS: Quat = (-0.5, 0.5, 0.5, 0.5)
# SteamVR's neutral controller pose is used directly. The physical neutral is
# established at runtime because direct HID starts from an arbitrary pose.
JOYCON_MODEL_OFFSET: Quat = (0.0, 0.0, 0.0, 1.0)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_quat(q: Quat) -> Quat:
    x, y, z, w = q
    mag = math.sqrt(x * x + y * y + z * z + w * w)
    if mag == 0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / mag, y / mag, z / mag, w / mag)


def quat_to_euler_deg(q: Quat) -> Tuple[float, float, float]:
    """Convert quaternion to roll/pitch/yaw in degrees."""
    x, y, z, w = normalize_quat(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def smooth_quat(prev: Optional[Quat], current: Quat, alpha: float) -> Quat:
    """Simple exponential smoothing for quaternions.

    This keeps the output stable enough for VR hand rotation.
    """
    if prev is None:
        return normalize_quat(current)

    px, py, pz, pw = prev
    cx, cy, cz, cw = current
    # Shortest-path blend
    dot = px * cx + py * cy + pz * cz + pw * cw
    if dot < 0.0:
        cx, cy, cz, cw = (-cx, -cy, -cz, -cw)

    blended = (
        px * (1.0 - alpha) + cx * alpha,
        py * (1.0 - alpha) + cy * alpha,
        pz * (1.0 - alpha) + cz * alpha,
        pw * (1.0 - alpha) + cw * alpha,
    )
    return normalize_quat(blended)


def slerp_quat(a: Quat, b: Quat, alpha: float) -> Quat:
    """Spherical interpolation with shortest-path quaternion handling."""
    ax, ay, az, aw = normalize_quat(a)
    bx, by, bz, bw = normalize_quat(b)
    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        return normalize_quat((
            ax + alpha * (bx - ax),
            ay + alpha * (by - ay),
            az + alpha * (bz - az),
            aw + alpha * (bw - aw),
        ))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    scale_a = math.sin((1.0 - alpha) * theta) / sin_theta
    scale_b = math.sin(alpha * theta) / sin_theta
    return normalize_quat((
        ax * scale_a + bx * scale_b,
        ay * scale_a + by * scale_b,
        az * scale_a + bz * scale_b,
        aw * scale_a + bw * scale_b,
    ))


def quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inverse(q: Quat) -> Quat:
    x, y, z, w = normalize_quat(q)
    return (-x, -y, -z, w)


def joycon_to_openvr_quat(q: Quat) -> Quat:
    axis_converted = quat_multiply(
        quat_multiply(JOYCON_AXIS_BASIS, normalize_quat(q)),
        quat_inverse(JOYCON_AXIS_BASIS),
    )
    # Direct-HID's normalized IMU frame reports roll with the opposite sign
    # from the in-game hand. Correct Z independently, based on the measured
    # 45-degree roll test, without changing pitch or yaw.
    axis_converted = (
        axis_converted[0],
        axis_converted[1],
        -axis_converted[2],
        axis_converted[3],
    )
    return normalize_quat(quat_multiply(axis_converted, JOYCON_MODEL_OFFSET))


def vmc_to_openvr_quat(q: Quat) -> Quat:
    """Reflect Unity/VMC +Z-forward rotations into OpenVR -Z-forward space."""
    x, y, z, w = normalize_quat(q)
    return normalize_quat((-x, -y, z, w))


def integrate_gyro(q: Quat, gyro_dps: Tuple[float, float, float], dt: float) -> Quat:
    """Integrate body-frame gyro rates into an x,y,z,w quaternion."""
    gx, gy, gz = (math.radians(value) for value in gyro_dps)
    omega = (gx, gy, gz, 0.0)
    derivative = quat_multiply(q, omega)
    return normalize_quat(tuple(q[i] + 0.5 * derivative[i] * dt for i in range(4)))  # type: ignore[return-value]


def rotate_vector(q: Quat, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    rotated = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (rotated[0], rotated[1], rotated[2])


def add_vectors(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


@dataclasses.dataclass
class PoseSample:
    device: str
    quat: Quat
    buttons: Dict[str, bool]
    stick: Tuple[float, float]
    timestamp: float
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_dict(cls, raw: Dict[str, object]) -> "PoseSample":
        device = str(raw.get("device", "unknown"))
        quat_raw = (
            raw.get("quat")
            or raw.get("quaternion")
            or raw.get("orientation")
            or raw.get("rot")
            or [0.0, 0.0, 0.0, 1.0]
        )
        if not isinstance(quat_raw, list) or len(quat_raw) != 4:
            raise ValueError("quat must be a 4-element list")
        quat = tuple(float(v) for v in quat_raw)  # type: ignore[assignment]

        buttons_raw = raw.get("buttons", {})
        buttons: Dict[str, bool] = {}
        if isinstance(buttons_raw, dict):
            for key, value in buttons_raw.items():
                buttons[str(key)] = bool(value)

        stick_raw = raw.get("stick", [0.0, 0.0])
        if not isinstance(stick_raw, list) or len(stick_raw) != 2:
            raise ValueError("stick must be a 2-element list")
        stick = (clamp(float(stick_raw[0]), -1.0, 1.0), clamp(float(stick_raw[1]), -1.0, 1.0))

        timestamp = float(raw.get("timestamp", time.time()))
        position_raw = raw.get("position", [0.0, 0.0, 0.0])
        if not isinstance(position_raw, list) or len(position_raw) != 3:
            position_raw = [0.0, 0.0, 0.0]
        position = tuple(float(v) for v in position_raw)
        return cls(device=device, quat=quat, buttons=buttons, stick=stick, timestamp=timestamp, position=position)


class FusionState:
    """Keep Joy-Con input and VMC reference pose in separate channels."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.joycon: Dict[str, PoseSample] = {}
        self.vmc: Dict[str, PoseSample] = {}

    def update_joycon(self, sample: PoseSample) -> None:
        with self.lock:
            self.joycon[sample.device] = sample

    def update_vmc(self, sample: PoseSample) -> None:
        with self.lock:
            self.vmc[sample.device] = sample

    def snapshot(self) -> Dict[str, Tuple[PoseSample, Optional[PoseSample]]]:
        with self.lock:
            # Joy-Con rotation must continue even when the camera/VMC stream
            # is stopped. VMC position is an optional overlay, not a gate.
            devices = set(self.joycon)
            return {device: (self.joycon[device], self.vmc.get(device)) for device in devices}


class VmcRotationCorrector:
    """Use VMC as a slow drift reference while preserving Joy-Con motion."""

    def __init__(self, time_constant: float = 6.0) -> None:
        self.time_constant = time_constant
        self.joycon_reference: Dict[str, Quat] = {}
        self.vmc_reference: Dict[str, Quat] = {}
        self.filtered_error: Dict[str, Quat] = {}
        self.last_time: Dict[str, float] = {}

    def reset(self, device: str) -> None:
        self.joycon_reference.pop(device, None)
        self.vmc_reference.pop(device, None)
        self.filtered_error.pop(device, None)
        self.last_time.pop(device, None)

    def correct(self, device: str, joycon: Quat, vmc: Optional[PoseSample], now: float) -> Quat:
        previous_error = self.filtered_error.get(device, (0.0, 0.0, 0.0, 1.0))
        if vmc is None or now - vmc.timestamp > 0.5:
            return normalize_quat(quat_multiply(previous_error, joycon))

        vmc_quat = vmc_to_openvr_quat(vmc.quat)
        if device not in self.joycon_reference:
            self.joycon_reference[device] = joycon
            self.vmc_reference[device] = vmc_quat
            self.filtered_error[device] = (0.0, 0.0, 0.0, 1.0)
            self.last_time[device] = now
            return joycon

        vmc_delta = quat_multiply(vmc_quat, quat_inverse(self.vmc_reference[device]))
        target = normalize_quat(quat_multiply(vmc_delta, self.joycon_reference[device]))
        target_error = normalize_quat(quat_multiply(target, quat_inverse(joycon)))
        dt = clamp(now - self.last_time.get(device, now), 0.0, 0.1)
        alpha = 1.0 - math.exp(-dt / self.time_constant)
        filtered_error = slerp_quat(previous_error, target_error, alpha)
        self.filtered_error[device] = filtered_error
        self.last_time[device] = now
        return normalize_quat(quat_multiply(filtered_error, joycon))


class JoyconRecenter:
    """Convert arbitrary direct-HID startup orientation into a neutral pose."""

    def __init__(self) -> None:
        self.reference: Dict[str, Quat] = {}
        self.combo_down: Dict[str, bool] = {}

    def apply(
        self,
        device: str,
        orientation: Quat,
        buttons: Dict[str, bool],
    ) -> Tuple[Quat, bool]:
        combo = bool(buttons.get("stick_click")) and bool(
            buttons.get("minus") if device == "left" else buttons.get("plus")
        )
        recentered = False
        if device not in self.reference or (combo and not self.combo_down.get(device, False)):
            self.reference[device] = orientation
            recentered = combo
        self.combo_down[device] = combo
        relative = quat_multiply(orientation, quat_inverse(self.reference[device]))
        return normalize_quat(relative), recentered


def pose_sample_from_event(raw: Dict[str, object]) -> PoseSample:
    """Normalize controller events from a local port into PoseSample objects.

    The 127.0.0.1:26760 source may expose slightly different keys depending on
    the upstream process. We accept the common shapes here and fall back to a
    neutral pose when a quaternion is missing.
    """
    device = str(raw.get("device") or raw.get("hand") or raw.get("side") or "unknown")

    quat_raw = (
        raw.get("quat")
        or raw.get("quaternion")
        or raw.get("orientation")
        or raw.get("rotation")
        or raw.get("rot")
    )
    if isinstance(quat_raw, dict):
        quat_raw = [quat_raw.get("x", 0.0), quat_raw.get("y", 0.0), quat_raw.get("z", 0.0), quat_raw.get("w", 1.0)]
    if not isinstance(quat_raw, list) or len(quat_raw) != 4:
        quat_raw = [0.0, 0.0, 0.0, 1.0]

    buttons_raw = raw.get("buttons")
    if not isinstance(buttons_raw, dict):
        buttons_raw = {
            key: raw.get(key)
            for key in ("a", "b", "x", "y", "l", "r", "zl", "zr", "plus", "minus", "stick", "home", "capture")
            if key in raw
        }

    stick_raw = raw.get("stick") or raw.get("analog_stick") or raw.get("left_stick") or [0.0, 0.0]
    if isinstance(stick_raw, dict):
        stick_raw = [stick_raw.get("x", 0.0), stick_raw.get("y", 0.0)]

    normalized = {
        "device": device,
        "quat": quat_raw,
        "buttons": buttons_raw,
        "stick": stick_raw,
        "timestamp": raw.get("timestamp", time.time()),
        "position": raw.get("position", [0.0, 0.0, 0.0]),
    }
    return PoseSample.from_dict(normalized)


class PoseBroadcaster:
    def __init__(self, udp_host: Optional[str], udp_port: Optional[int]) -> None:
        self.sock = None
        self.target = None
        self.last_print_at = 0.0
        if udp_host and udp_port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.target = (udp_host, udp_port)

    def send(self, packet: Dict[str, object], *, quiet: bool = False) -> None:
        line = json.dumps(packet, ensure_ascii=False)
        if not quiet:
            now = time.time()
            if now - self.last_print_at >= 0.25:
                self.last_print_at = now
                print(line, flush=True)
        if self.sock and self.target:
            self.sock.sendto(line.encode("utf-8"), self.target)


class PoseProcessor:
    def __init__(self, alpha: float = 0.25) -> None:
        self.alpha = alpha
        self.last_quat: Dict[str, Quat] = {}

    def process(self, sample: PoseSample) -> Dict[str, object]:
        prev = self.last_quat.get(sample.device)
        smoothed = smooth_quat(prev, sample.quat, self.alpha)
        self.last_quat[sample.device] = smoothed
        roll, pitch, yaw = quat_to_euler_deg(smoothed)
        return {
            "device": sample.device,
            "timestamp": sample.timestamp,
            "quat": [smoothed[0], smoothed[1], smoothed[2], smoothed[3]],
            "rotation_source": "joycon",
            "euler_deg": [roll, pitch, yaw],
            "buttons": sample.buttons,
            "stick": [sample.stick[0], sample.stick[1]],
            "position": [sample.position[0], sample.position[1], sample.position[2]],
        }


def build_steamvr_packet(packet: Dict[str, object]) -> Dict[str, object]:
    """Build a SteamVR-friendly controller payload.

    This is not a native OpenVR driver. It is the wire format we want a driver
    or adapter process to consume.
    """
    buttons = packet.get("buttons", {})
    stick = packet.get("stick", [0.0, 0.0])
    quat = packet.get("quat", [0.0, 0.0, 0.0, 1.0])
    euler = packet.get("euler_deg", [0.0, 0.0, 0.0])

    trigger = 1.0 if bool(buttons.get("zr") or buttons.get("r") or buttons.get("a")) else 0.0
    grip = 1.0 if bool(buttons.get("zl") or buttons.get("l") or buttons.get("b")) else 0.0
    menu = bool(buttons.get("plus") or buttons.get("minus") or buttons.get("home"))
    app = bool(buttons.get("capture") or buttons.get("x") or buttons.get("y"))
    stick_click = bool(buttons.get("stick_click"))

    return {
        "device": packet.get("device", "unknown"),
        "timestamp": packet.get("timestamp", time.time()),
        "role": "right" if packet.get("device") == "right" else "left",
        "pose": {
            "quat": quat,
            "euler_deg": euler,
            "position": packet.get("position", [0.0, 0.0, 0.0]),
        },
        "inputs": {
            "trigger": trigger,
            "grip": grip,
            "menu": menu,
            "application_menu": app,
            "stick_click": stick_click,
            "stick_touch": True,
            "stick": {
                "x": float(stick[0]) if isinstance(stick, list) and len(stick) > 0 else 0.0,
                "y": float(stick[1]) if isinstance(stick, list) and len(stick) > 1 else 0.0,
            },
        },
        "raw": packet,
    }


class PacketSink:
    def __init__(self, host: Optional[str], port: Optional[int]) -> None:
        self.sock = None
        self.target = None
        if host and port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.target = (host, port)

    def send(self, packet: Dict[str, object]) -> None:
        if self.sock and self.target:
            self.sock.sendto(json.dumps(packet, ensure_ascii=False).encode("utf-8"), self.target)


def osc_string(value: str) -> bytes:
    data = value.encode("utf-8") + b"\x00"
    return data + (b"\x00" * ((4 - len(data) % 4) % 4))


def osc_message(address: str, type_tags: str, *values: object) -> bytes:
    payload = osc_string(address) + osc_string(type_tags)
    import struct

    for tag, value in zip(type_tags[1:], values):
        if tag == "s":
            payload += osc_string(str(value))
        elif tag == "f":
            payload += struct.pack(">f", float(value))
        elif tag == "i":
            payload += struct.pack(">i", int(value))
    return payload


class VmcSink:
    """Send controller and hand poses using the VMC OSC protocol."""

    def __init__(self, host: Optional[str], port: Optional[int]) -> None:
        self.sock = None
        self.target = None
        if host and port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.target = (host, port)

    def send(self, packet: Dict[str, object]) -> None:
        if not self.sock or not self.target:
            return
        device = str(packet.get("device", "unknown"))
        quat = packet.get("quat", [0.0, 0.0, 0.0, 1.0])
        if not isinstance(quat, list) or len(quat) != 4:
            return
        x, y, z, w = (float(value) for value in quat)
        serial = f"joycon-{device}"
        bone = "LeftHand" if device == "left" else "RightHand"
        args = (serial, 0.0, 0.0, 0.0, x, y, z, w)
        self.sock.sendto(osc_message("/VMC/Ext/Con/Pos", ",sfffffff", *args), self.target)
        self.sock.sendto(osc_message("/VMC/Ext/Bone/Pos", ",sfffffff", bone, 0.0, 0.0, 0.0, x, y, z, w), self.target)


def iter_stdin_json() -> Iterator[PoseSample]:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        yield PoseSample.from_dict(json.loads(line))


def iter_udp_json(port: int) -> Iterator[PoseSample]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    while True:
        data, _addr = sock.recvfrom(65535)
        yield PoseSample.from_dict(json.loads(data.decode("utf-8")))


class BetterJoyDsuClient:
    """Read BetterJoy's Cemuhook/DSU motion server without binding its port."""

    DSU_VERSION = 1001
    VERSION = 0x100000
    LIST_PORTS = 0x100001
    PAD_DATA = 0x100002

    def __init__(self, host: str, port: int, *, trace: bool = False) -> None:
        self.target = (host, port)
        self.trace = trace
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.client_id = (int(time.time() * 1000000) ^ id(self)) & 0xFFFFFFFF
        self.quats: Dict[int, Quat] = {}
        self.last_time: Dict[int, float] = {}
        self.device_names: Dict[int, str] = {}
        self.last_trace_at = 0.0

    def close(self) -> None:
        self.sock.close()

    def _packet(self, message_type: int, payload: bytes = b"") -> bytes:
        # DSU's size includes the message type (the 16-byte header does not).
        packet = bytearray(struct.pack("<4sHHIII", b"DSUC", self.DSU_VERSION, 4 + len(payload), 0, self.client_id, message_type) + payload)
        packet[8:12] = struct.pack("<I", zlib.crc32(packet) & 0xFFFFFFFF)
        return bytes(packet)

    def subscribe(self) -> None:
        self.sock.sendto(self._packet(self.VERSION), self.target)
        self.sock.sendto(self._packet(self.LIST_PORTS, struct.pack("<iBBBB", 4, 0, 1, 2, 3)), self.target)
        self.sock.sendto(self._packet(self.PAD_DATA, b"\x00\x00" + b"\x00" * 6), self.target)
        if self.trace:
            print(f"[dsu] listening from {self.target[0]}:{self.target[1]} via local ephemeral port", flush=True)

    @staticmethod
    def _device_name(pad_id: int) -> str:
        # BetterJoy normally exposes the paired Joy-Cons as slots 0 and 1.
        return "left" if pad_id == 0 else "right" if pad_id == 1 else f"pad{pad_id}"

    def _sample(self, data: bytes) -> Optional[PoseSample]:
        if len(data) < 100 or data[:4] != b"DSUS":
            return None
        packet_size = struct.unpack_from("<H", data, 6)[0]
        if packet_size + 16 > len(data) or struct.unpack_from("<I", data, 16)[0] != self.PAD_DATA:
            return None
        pad_id = data[20]
        connected = data[21]
        if not connected:
            return None
        buttons1, buttons2 = data[36], data[37]
        stick_x, stick_y = data[40], data[41]
        if pad_id == 0:
            sx, sy = stick_x, stick_y
        else:
            sx, sy = data[42], data[43]
        gyro = struct.unpack_from("<fff", data, 88)
        now = time.perf_counter()
        previous = self.quats.get(pad_id, (0.0, 0.0, 0.0, 1.0))
        previous_time = self.last_time.get(pad_id, now)
        dt = max(0.0, min(now - previous_time, 0.05))
        current = integrate_gyro(previous, gyro, dt)
        self.quats[pad_id] = current
        self.last_time[pad_id] = now
        device = self.device_names.setdefault(pad_id, self._device_name(pad_id))
        buttons = {
            "y": bool(buttons2 & 0x10), "b": bool(buttons2 & 0x40),
            "a": bool(buttons2 & 0x20), "x": bool(buttons2 & 0x80),
            "r": bool(buttons2 & 0x08), "l": bool(buttons2 & 0x04),
            "zr": bool(buttons2 & 0x02), "zl": bool(buttons2 & 0x01),
            "plus": bool(buttons1 & 0x08), "minus": bool(buttons1 & 0x01),
            "home": bool(data[38]), "capture": bool(buttons1 & 0x01),
            "stick_click": bool(buttons1 & (0x02 if pad_id == 0 else 0x04)),
        }
        stick = (clamp((sx - 128.0) / 127.0, -1.0, 1.0), clamp((128.0 - sy) / 127.0, -1.0, 1.0))
        if self.trace and time.monotonic() - self.last_trace_at >= 0.5:
            self.last_trace_at = time.monotonic()
            print(f"[dsu] pad={pad_id} device={device} gyro=({gyro[0]:.1f},{gyro[1]:.1f},{gyro[2]:.1f})", flush=True)
        return PoseSample(device=device, quat=current, buttons=buttons, stick=stick, timestamp=time.time())

    def samples(self) -> Iterator[PoseSample]:
        self.subscribe()
        next_refresh = time.monotonic() + 2.0
        try:
            while True:
                if time.monotonic() >= next_refresh:
                    self.sock.sendto(self._packet(self.PAD_DATA, b"\x00\x00" + b"\x00" * 6), self.target)
                    next_refresh = time.monotonic() + 2.0
                try:
                    data, _addr = self.sock.recvfrom(2048)
                except socket.timeout:
                    continue
                sample = self._sample(data)
                if sample:
                    yield sample
        finally:
            self.close()


def osc_read_string(data: bytes, offset: int) -> Tuple[str, int]:
    end = data.find(b"\x00", offset)
    if end < 0:
        raise ValueError("invalid OSC string")
    value = data[offset:end].decode("utf-8", errors="replace")
    return value, (end + 4) & ~3


def parse_osc_message(data: bytes) -> Tuple[str, list]:
    import struct

    address, offset = osc_read_string(data, 0)
    tags, offset = osc_read_string(data, offset)
    values = []
    for tag in tags[1:]:
        if tag == "s":
            value, offset = osc_read_string(data, offset)
            values.append(value)
        elif tag == "f":
            if offset + 4 > len(data):
                raise ValueError("invalid OSC float")
            values.append(struct.unpack(">f", data[offset:offset + 4])[0])
            offset += 4
        elif tag == "i":
            if offset + 4 > len(data):
                raise ValueError("invalid OSC int")
            values.append(struct.unpack(">i", data[offset:offset + 4])[0])
            offset += 4
        else:
            raise ValueError(f"unsupported OSC type {tag}")
    return address, values


def iter_osc_messages(data: bytes) -> Iterator[Tuple[str, list]]:
    """Yield OSC messages from either a packet or an OSC bundle."""
    if data.startswith(b"#bundle"):
        _bundle_name, offset = osc_read_string(data, 0)
        if offset + 8 > len(data):
            raise ValueError("invalid OSC bundle timetag")
        offset += 8
        while offset + 4 <= len(data):
            import struct

            size = struct.unpack(">i", data[offset:offset + 4])[0]
            offset += 4
            if size < 0 or offset + size > len(data):
                raise ValueError("invalid OSC bundle element")
            yield from iter_osc_messages(data[offset:offset + size])
            offset += size
    else:
        yield parse_osc_message(data)


def iter_vmc_udp(port: int, *, trace: bool = False) -> Iterator[PoseSample]:
    """Receive XR Animator VMC controller/bone poses over OSC."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    if trace:
        print(f"[vmc] listening on 127.0.0.1:{port}", flush=True)
    last_trace_at: Dict[str, float] = {}
    last_lowerarm_quat: Dict[str, Quat] = {}
    last_hand_position: Dict[str, Tuple[float, float, float]] = {}
    bone_local: Dict[str, Tuple[Tuple[float, float, float], Quat]] = {}

    def chain_world_transform(
        chain: Tuple[str, ...],
    ) -> Optional[Tuple[Tuple[float, float, float], Quat]]:
        if any(name not in bone_local for name in chain):
            return None
        world_position, world_rotation = bone_local[chain[0]]
        for name in chain[1:]:
            local_position, local_rotation = bone_local[name]
            world_position = add_vectors(world_position, rotate_vector(world_rotation, local_position))
            world_rotation = normalize_quat(quat_multiply(world_rotation, local_rotation))
        return world_position, world_rotation

    def hand_world_pose(
        device: str,
    ) -> Optional[Tuple[Tuple[float, float, float], Quat]]:
        side = "left" if device == "left" else "right"
        chain = ("hips", "spine", "chest", "upperchest", f"{side}shoulder", f"{side}upperarm", f"{side}lowerarm", f"{side}hand")
        hand_transform = chain_world_transform(chain)
        head_chain = ("hips", "spine", "chest", "upperchest", "neck", "head")
        head_transform = chain_world_transform(head_chain)
        if hand_transform is None or head_transform is None:
            return None
        hand_position, hand_rotation = hand_transform
        head_position, _head_rotation = head_transform
        # SteamVR's HMD remains in its own tracking origin. Remove VMC's
        # global body translation so the hands stay attached to that HMD.
        # VMC/Unity uses +Z forward; OpenVR uses -Z forward.
        return (
            (
                hand_position[0] - head_position[0],
                hand_position[1] - head_position[1],
                -(hand_position[2] - head_position[2]),
            ),
            hand_rotation,
        )

    while True:
        data, _addr = sock.recvfrom(65535)
        try:
            messages = iter_osc_messages(data)
            for address, values in messages:
                if address not in ("/VMC/Ext/Con/Pos", "/VMC/Ext/Tra/Pos", "/VMC/Ext/Bone/Pos"):
                    continue
                if len(values) < 8:
                    continue
                name_raw = str(values[0])
                name = name_raw.lower()
                if address == "/VMC/Ext/Bone/Pos":
                    bone_local[name] = (
                        (float(values[1]), float(values[2]), float(values[3])),
                        normalize_quat((float(values[4]), float(values[5]), float(values[6]), float(values[7]))),
                    )
                    is_lowerarm = name in ("leftlowerarm", "rightlowerarm") or name_raw in ("左前腕", "右前腕")
                    is_hand = name in ("lefthand", "righthand", "leftwrist", "rightwrist") or name_raw in ("左手", "右手", "左手首", "右手首")
                    if not (is_lowerarm or is_hand):
                        continue
                if (
                    "left" in name
                    or "左" in name
                    or name in ("lefthand", "l")
                ):
                    device = "left"
                elif (
                    "right" in name
                    or "右" in name
                    or name in ("righthand", "r")
                ):
                    device = "right"
                else:
                    if trace and address == "/VMC/Ext/Bone/Pos":
                        print(f"[vmc] ignored bone={values[0]!r}", flush=True)
                    continue
                position = (float(values[1]), float(values[2]), float(values[3]))
                quat = [float(values[4]), float(values[5]), float(values[6]), float(values[7])]
                if address == "/VMC/Ext/Bone/Pos":
                    if is_lowerarm:
                        last_lowerarm_quat[device] = tuple(quat)
                    if is_hand:
                        last_hand_position[device] = position
                    # Rotation is Joy-Con-owned later; VMC only supplies position.
                    hand_pose = hand_world_pose(device)
                    if hand_pose is not None:
                        output_position, output_quat = hand_pose
                    else:
                        output_quat = last_lowerarm_quat.get(device, tuple(quat))
                        output_position = last_hand_position.get(device, position)
                else:
                    output_quat = tuple(quat)
                    output_position = position
                now = time.time()
                if trace and now - last_trace_at.get(device, 0.0) >= 0.25:
                    last_trace_at[device] = now
                    print(f"[vmc] {address} {device} quat={quat}", flush=True)
                yield PoseSample(device=device, quat=output_quat, buttons={}, stick=(0.0, 0.0), timestamp=now, position=output_position)
        except (ValueError, UnicodeError, IndexError):
            continue


def iter_tcp_json(host: str, port: int, *, trace: bool = False) -> Iterator[PoseSample]:
    while True:
        try:
            if trace:
                print(f"[tcp] connecting to {host}:{port}", flush=True)
            with socket.create_connection((host, port), timeout=5.0) as sock:
                if trace:
                    print("[tcp] connected", flush=True)
                file = sock.makefile("r", encoding="utf-8", newline="\n")
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    if trace:
                        print(f"[tcp] received {line[:240]}", flush=True)
                    yield pose_sample_from_event(json.loads(line))
                if trace:
                    print("[tcp] connection closed", flush=True)
        except OSError as exc:
            if trace:
                print(f"[tcp] waiting: {exc}", flush=True)
            time.sleep(1.0)


def iter_fused_inputs(
    joycon_host: str,
    joycon_port: int,
    vmc_port: int,
    *,
    trace: bool = False,
) -> Iterator[PoseSample]:
    """Use direct Joy-Con HID orientation/input and VMC hand position."""
    state = FusionState()
    corrector = VmcRotationCorrector(time_constant=6.0)
    recenter = JoyconRecenter()

    def consume_joycon() -> None:
        try:
            for sample in iter_direct_joycon(trace=trace):
                state.update_joycon(sample)
        except Exception as exc:
            if trace:
                print(f"[joycon] direct HID failed ({exc}); falling back to DSU", flush=True)
            for sample in BetterJoyDsuClient(joycon_host, joycon_port, trace=trace).samples():
                state.update_joycon(sample)

    def consume_vmc() -> None:
        for sample in iter_vmc_udp(vmc_port, trace=trace):
            state.update_vmc(sample)

    threading.Thread(target=consume_joycon, daemon=True, name="joycon-input").start()
    threading.Thread(target=consume_vmc, daemon=True, name="vmc-input").start()
    if trace:
        print(
            f"[fusion] direct Joy-Con HID + VMC UDP 127.0.0.1:{vmc_port}",
            flush=True,
        )

    while True:
        for device, (joycon, vmc) in state.snapshot().items():
            now = time.time()
            absolute_quat = joycon_to_openvr_quat(joycon.quat)
            joycon_quat, was_recentered = recenter.apply(
                device, absolute_quat, joycon.buttons
            )
            if was_recentered:
                corrector.reset(device)
                if trace:
                    print(f"[recenter] device={device} neutral pose saved", flush=True)
            yield PoseSample(
                device=device,
                quat=corrector.correct(device, joycon_quat, vmc, now),
                buttons=joycon.buttons,
                stick=joycon.stick,
                timestamp=now,
                position=vmc.position if vmc is not None else (0.0, 0.0, 0.0),
            )
        time.sleep(1.0 / 120.0)


def iter_dsu_inputs(host: str, port: int, *, trace: bool = False) -> Iterator[PoseSample]:
    """Read only BetterJoy DSU data, for testing without the camera/VMC."""
    positions = {"left": (-0.22, -0.35, 0.0), "right": (0.22, -0.35, 0.0)}
    for sample in BetterJoyDsuClient(host, port, trace=trace).samples():
        yield dataclasses.replace(
            sample,
            quat=joycon_to_openvr_quat(sample.quat),
            position=positions.get(sample.device, (0.0, -0.35, 0.0)),
        )


def iter_direct_joycon(*, trace: bool = False) -> Iterator[PoseSample]:
    """Read both paired Joy-Cons directly over Windows HID Bluetooth."""
    try:
        from pyjoycon.constants import JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID, JOYCON_VENDOR_ID
        from pyjoycon.gyro import GyroTrackingJoyCon
    except ImportError as exc:
        raise RuntimeError(
            "Direct Joy-Con input needs joycon-python, hidapi, and PyGLM. "
            "Install with: python -m pip install --user hidapi PyGLM "
            "git+https://github.com/tocoteron/joycon-python.git"
        ) from exc

    left = GyroTrackingJoyCon(JOYCON_VENDOR_ID, JOYCON_L_PRODUCT_ID)
    right = GyroTrackingJoyCon(JOYCON_VENDOR_ID, JOYCON_R_PRODUCT_ID)
    controllers = (("left", left), ("right", right))
    if trace:
        print("[joycon] direct Bluetooth HID connected: left, right", flush=True)

    def stick_value(value: int) -> float:
        return clamp((float(value) - 2048.0) / 1700.0, -1.0, 1.0)

    while True:
        for device, controller in controllers:
            q = controller.direction_Q
            status = controller.get_status()
            buttons = status.get("buttons", {})
            shared = buttons.get("shared", {}) if isinstance(buttons, dict) else {}
            side = buttons.get(device, {}) if isinstance(buttons, dict) else {}
            if not isinstance(shared, dict):
                shared = {}
            if not isinstance(side, dict):
                side = {}
            stick_key = "left" if device == "left" else "right"
            sticks = status.get("analog-sticks", {})
            stick = sticks.get(stick_key, {}) if isinstance(sticks, dict) else {}
            if not isinstance(stick, dict):
                stick = {}
            merged_buttons = {str(k): bool(v) for k, v in {**shared, **side}.items()}
            merged_buttons["stick_click"] = bool(
                shared.get("l-stick" if device == "left" else "r-stick", False)
            )
            yield PoseSample(
                device=device,
                quat=(float(q.x), float(q.y), float(q.z), float(q.w)),
                buttons=merged_buttons,
                stick=(
                    stick_value(int(stick.get("horizontal", 2048))),
                    stick_value(int(stick.get("vertical", 2048))),
                ),
                timestamp=time.time(),
            )
        time.sleep(1.0 / 120.0)


def iter_simulated() -> Iterator[PoseSample]:
    start = time.time()
    while True:
        t = time.time() - start
        # Gentle rotation and stick sweep for smoke tests.
        yaw = math.sin(t * 1.3) * 0.8
        pitch = math.sin(t * 0.7) * 0.25
        roll = math.sin(t * 0.5) * 0.15
        q = euler_deg_to_quat(roll * 180.0 / math.pi, pitch * 180.0 / math.pi, yaw * 180.0 / math.pi)
        yield PoseSample(
            device="right",
            quat=q,
            buttons={"a": int(math.sin(t * 2.0) > 0.7)},
            stick=(math.sin(t) * 0.8, math.cos(t * 0.7) * 0.8),
            timestamp=time.time(),
        )
        time.sleep(1.0 / 60.0)


def euler_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quat:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return normalize_quat((x, y, z, w))


def main() -> int:
    parser = argparse.ArgumentParser(description="Joy-Con pose bridge")
    parser.add_argument("--input", choices=["stdin", "tcp", "udp", "vmc", "fusion", "dsu", "direct", "sim"], default="sim")
    parser.add_argument("--tcp-host", default="127.0.0.1", help="TCP input host for --input tcp")
    parser.add_argument("--tcp-port", type=int, default=26760, help="TCP input port for --input tcp")
    parser.add_argument("--udp-port", type=int, default=39770, help="UDP output port")
    parser.add_argument("--udp-host", default="127.0.0.1", help="UDP output host")
    parser.add_argument("--steamvr-port", type=int, default=39772, help="UDP output port for SteamVR-ready packets")
    parser.add_argument("--vmc-host", default="127.0.0.1", help="VMC OSC destination host")
    parser.add_argument("--vmc-port", type=int, default=0, help="VMC OSC destination port; 0 disables VMC output")
    parser.add_argument("--listen-port", type=int, default=39771, help="UDP input port for --input udp")
    parser.add_argument("--alpha", type=float, default=0.25, help="Quaternion smoothing factor")
    parser.add_argument("--quiet", action="store_true", help="Reduce console output and only keep UDP forwarding")
    parser.add_argument("--trace", action="store_true", help="Print TCP connection and received-packet diagnostics")
    args = parser.parse_args()

    broadcaster = PoseBroadcaster(args.udp_host, args.udp_port)
    steamvr_sink = PacketSink(args.udp_host, args.steamvr_port)
    vmc_sink = VmcSink(args.vmc_host, args.vmc_port)
    processor = PoseProcessor(alpha=clamp(args.alpha, 0.01, 1.0))

    if args.input == "stdin":
        source = iter_stdin_json()
    elif args.input == "tcp":
        source = iter_tcp_json(args.tcp_host, args.tcp_port, trace=args.trace)
    elif args.input == "udp":
        source = iter_udp_json(args.listen_port)
    elif args.input == "vmc":
        source = iter_vmc_udp(args.listen_port, trace=args.trace)
    elif args.input == "fusion":
        source = iter_fused_inputs(args.tcp_host, args.tcp_port, args.listen_port, trace=args.trace)
    elif args.input == "dsu":
        source = iter_dsu_inputs(args.tcp_host, args.tcp_port, trace=args.trace)
    elif args.input == "direct":
        source = iter_direct_joycon(trace=args.trace)
    else:
        source = iter_simulated()

    last_pose_trace_at: Dict[str, float] = {}
    for sample in source:
        packet = processor.process(sample)
        now = time.time()
        if args.trace and now - last_pose_trace_at.get(sample.device, 0.0) >= 0.25:
            last_pose_trace_at[sample.device] = now
            print(
                f"[pose] t={now:.3f} device={sample.device} quat=({packet['quat'][0]:.4f}, "
                f"{packet['quat'][1]:.4f}, {packet['quat'][2]:.4f}, {packet['quat'][3]:.4f})",
                f" rot=joycon+vmc-drift-correction pos_source=vmc-head-relative "
                f"pos=({packet['position'][0]:.3f}, {packet['position'][1]:.3f}, {packet['position'][2]:.3f})",
                flush=True,
            )
        broadcaster.send(packet, quiet=args.quiet)
        steamvr_sink.send(build_steamvr_packet(packet))
        vmc_sink.send(packet)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
