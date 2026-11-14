# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Live state of the MOCAP4ROS2 systems, kept by a background ROS executor."""

from dataclasses import dataclass, field
import threading
import time
from typing import Dict, List, Optional

from diagnostic_msgs.msg import DiagnosticArray
from mocap4r2_interfaces.msg import Markers, MocapSystemInfo, RigidBodies
from mocap4r2_tools.client import CONTROL_QOS, SYSTEMS_QOS, SYSTEMS_TOPIC
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DATA_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


@dataclass
class SystemStats:
    """Frame statistics of a system, from its diagnostics."""

    level: int = 0
    message: str = ''
    rate: float = 0.0
    latency_ms: float = 0.0
    dropped: int = 0
    total_dropped: int = 0
    total_frames: int = 0
    tracking: Dict[str, float] = field(default_factory=dict)
    server: str = ''
    connected: bool = False
    stamp: float = 0.0


@dataclass
class BodiesView:
    """Last rigid bodies and marker counts of the watched system."""

    bodies: Optional[RigidBodies] = None
    labeled: int = 0
    unlabeled: int = 0
    occluded: int = 0
    stamp: float = 0.0


def parse_diagnostic(status) -> Optional[tuple]:
    """Return (system id, SystemStats) of a MOCAP4ROS2 diagnostic status, or None."""
    values = {kv.key: kv.value for kv in status.values}
    system = values.get('system')
    if not system:
        return None
    s = SystemStats(level=int.from_bytes(status.level, 'little')
                    if isinstance(status.level, bytes) else int(status.level),
                    message=status.message, stamp=time.monotonic())

    def num(key, kind=float):
        try:
            return kind(float(values.get(key, 0)))
        except ValueError:
            return kind(0)
    s.rate = num('rate_hz')
    s.latency_ms = num('latency_mean_ms')
    s.dropped = num('dropped', int)
    s.total_dropped = num('total_dropped', int)
    s.total_frames = num('total_frames', int)
    s.server = values.get('server', '')
    s.connected = values.get('connected', 'false') == 'true'
    for k, v in values.items():
        if k.startswith('tracking/'):
            try:
                s.tracking[k[len('tracking/'):]] = float(v)
            except ValueError:
                pass
    return system, s


class MocapMonitor:
    """Subscribes to systems, diagnostics and the data of one watched system."""

    def __init__(self, node_name: str = 'mocap4r2_monitor'):
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self.node: Node = rclpy.create_node('_' + node_name)
        self._lock = threading.Lock()
        self._systems: Dict[str, MocapSystemInfo] = {}
        self._stats: Dict[str, SystemStats] = {}
        self._view = BodiesView()
        self._watched: Optional[str] = None
        self._data_subs: List = []
        self.node.create_subscription(
            MocapSystemInfo, SYSTEMS_TOPIC, self._on_system, SYSTEMS_QOS)
        self.node.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, CONTROL_QOS)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:  # noqa: B902  shutdown races
                break

    def close(self):
        self._executor.shutdown()
        self.node.destroy_node()
        if self._own_context:
            rclpy.try_shutdown()

    def _on_system(self, msg: MocapSystemInfo):
        with self._lock:
            self._systems[msg.system] = msg

    def _on_diagnostics(self, msg: DiagnosticArray):
        with self._lock:
            for status in msg.status:
                parsed = parse_diagnostic(status)
                if parsed:
                    self._stats[parsed[0]] = parsed[1]

    def _on_bodies(self, msg: RigidBodies):
        with self._lock:
            self._view.bodies = msg
            self._view.stamp = time.monotonic()

    def _on_markers(self, msg: Markers):
        labeled = unlabeled = occluded = 0
        for m in msg.markers:
            if m.occluded:
                occluded += 1
            elif m.id == m.UNKNOWN_ID and not m.name:
                unlabeled += 1
            else:
                labeled += 1
        with self._lock:
            self._view.labeled, self._view.unlabeled = labeled, unlabeled
            self._view.occluded = occluded

    def watch(self, system: Optional[str]):
        """Subscribe to the rigid bodies and markers announced by a system."""
        with self._lock:
            if system == self._watched:
                return
            info = self._systems.get(system) if system else None
            self._watched = system
            self._view = BodiesView()
        for sub in self._data_subs:
            self.node.destroy_subscription(sub)
        self._data_subs = []
        if not info:
            return
        for topic, kind in zip(info.topics, info.topic_types):
            if kind.endswith('/RigidBodies'):
                self._data_subs.append(self.node.create_subscription(
                    RigidBodies, topic, self._on_bodies, DATA_QOS))
            elif kind.endswith('/Markers'):
                self._data_subs.append(self.node.create_subscription(
                    Markers, topic, self._on_markers, DATA_QOS))

    def alive(self) -> set:
        names = set()
        for name, ns in self.node.get_node_names_and_namespaces():
            names.add(ns.rstrip('/') + '/' + name)
        return names

    def snapshot(self):
        """(systems, stats, bodies view) copies for the UI."""
        alive = self.alive()
        with self._lock:
            systems = {k: v for k, v in self._systems.items()
                       if k in alive and v.state.label != 'finalized'}
            return systems, dict(self._stats), self._view
