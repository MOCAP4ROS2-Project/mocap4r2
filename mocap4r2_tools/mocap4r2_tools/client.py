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

"""Python client of the MOCAP4ROS2 control plane, shared by the CLI and the TUI."""

from contextlib import contextmanager
from dataclasses import dataclass, field
import time
from typing import Dict, Iterable, List, Optional
import uuid

from mocap4r2_interfaces.msg import Control, ControlAck, MocapSystemInfo
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

CONTROL_TOPIC = '/mocap4r2/control'
CONTROL_ACK_TOPIC = '/mocap4r2/control_ack'
SYSTEMS_TOPIC = '/mocap4r2/systems'

SYSTEMS_QOS = QoSProfile(
    depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CONTROL_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class MocapError(Exception):
    """Error talking to MOCAP4ROS2 systems."""


@dataclass
class ControlResult:
    """Outcome of a START/STOP request."""

    request_id: str
    acks: Dict[str, ControlAck] = field(default_factory=dict)
    missing: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.missing and all(a.success for a in self.acks.values())


@contextmanager
def mocap_node(name: str = 'mocap4r2_cli', handle_signals: bool = True):
    """Create a hidden rclpy node for one command."""
    own_context = not rclpy.ok()
    if own_context:
        if handle_signals:
            rclpy.init()
        else:
            # The command handles Ctrl-C itself and still needs ROS afterwards.
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('_' + name + '_' + uuid.uuid4().hex[:8])
    try:
        yield node
    finally:
        node.destroy_node()
        if own_context:
            rclpy.try_shutdown()


class MocapClient:
    """Discover mocap systems and start/stop them."""

    def __init__(self, node: Node):
        self._node = node
        self._systems: Dict[str, MocapSystemInfo] = {}
        self._acks: List[ControlAck] = []
        self._systems_sub = node.create_subscription(
            MocapSystemInfo, SYSTEMS_TOPIC, self._on_system, SYSTEMS_QOS)
        self._ack_sub = node.create_subscription(
            ControlAck, CONTROL_ACK_TOPIC, self._acks.append, CONTROL_QOS)
        self._control_pub = node.create_publisher(Control, CONTROL_TOPIC, CONTROL_QOS)

    def _on_system(self, msg: MocapSystemInfo):
        self._systems[msg.system] = msg

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self._node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def spin_until(self, cond, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return cond()

    def systems(self, timeout: float = 1.0, alive_only: bool = False,
                wait_for: Iterable[str] = ()) -> Dict[str, MocapSystemInfo]:
        """
        Return the announced systems, keyed by fully qualified node name.

        Waits until no new announcement arrives for a while, at most timeout seconds;
        with wait_for, until those systems (full or plain names) are known.
        Finalized systems are skipped. A fresh node only receives announcements of
        live systems; long-lived nodes can drop systems whose node is gone with
        alive_only (it needs the ROS graph, which may be discovered later).
        """
        wanted = list(wait_for)
        start = time.monotonic()
        end = start + timeout
        last_count = -1
        stable_since = start
        while time.monotonic() < end:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            now = time.monotonic()
            if len(self._systems) != last_count:
                last_count = len(self._systems)
                stable_since = now
            elif wanted:
                if all(self._known(w) for w in wanted):
                    break
            elif last_count > 0 and now - stable_since > 0.6 and now - start > 1.0:
                break  # announcements settled (discovery may bring them one by one)
        systems = {k: v for k, v in self._systems.items() if v.state.label != 'finalized'}
        if alive_only:
            alive = {
                (ns.rstrip('/') + '/' + name) for name, ns in
                self._node.get_node_names_and_namespaces()}
            systems = {
                k: v for k, v in systems.items()
                if k in alive and v.state.label != 'finalized'}
        return systems

    def _known(self, name: str) -> bool:
        return name in self._systems or any(
            s.rsplit('/', 1)[-1] == name.strip('/') for s in self._systems)

    def resolve(self, names: Iterable[str], systems: Dict[str, MocapSystemInfo]) -> List[str]:
        """Map user names (fully qualified or unique plain names) to system ids."""
        resolved = []
        for name in names:
            if name in systems:
                resolved.append(name)
                continue
            matches = [s for s in systems if s.rsplit('/', 1)[-1] == name.strip('/')]
            if len(matches) == 1:
                resolved.append(matches[0])
            elif not matches:
                raise MocapError(f"unknown mocap system '{name}'")
            else:
                raise MocapError(f"'{name}' is ambiguous: {', '.join(sorted(matches))}")
        return resolved

    def control(
        self, command: int, targets: List[str], session_id: str = '',
        timeout: float = 5.0, systems: Optional[Dict[str, MocapSystemInfo]] = None
    ) -> ControlResult:
        """
        Send START/STOP to targets (all known systems if empty) and wait for the acks.

        The request is published only when every expected system listens to the
        control topic, so it cannot be lost to discovery.
        """
        if systems is None:
            systems = self.systems()
        expected = list(targets) if targets else list(systems)
        if not expected:
            raise MocapError('no mocap systems found')

        self.spin_until(
            lambda: self._control_pub.get_subscription_count() >= len(expected) and
            self._node.count_publishers(CONTROL_ACK_TOPIC) >= len(expected), timeout)

        msg = Control()
        msg.command = command
        msg.request_id = uuid.uuid4().hex
        msg.session_id = session_id
        msg.requester = self._node.get_fully_qualified_name()
        msg.systems = list(targets)
        msg.stamp = self._node.get_clock().now().to_msg()
        self._control_pub.publish(msg)

        result = ControlResult(request_id=msg.request_id)

        def done():
            for ack in self._acks:
                if ack.request_id == msg.request_id:
                    result.acks[ack.system] = ack
            return all(s in result.acks for s in expected)

        self.spin_until(done, timeout)
        result.missing = [s for s in expected if s not in result.acks]
        return result
