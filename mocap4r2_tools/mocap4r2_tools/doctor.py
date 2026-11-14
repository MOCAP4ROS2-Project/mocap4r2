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

"""Checks of a MOCAP4ROS2 setup (ros2 mocap4r2 doctor)."""

import ipaddress
import os
import socket
import subprocess
from typing import List, Tuple

OK, WARN, FAIL = 'ok', 'warn', 'fail'

ADDRESS_PARAMS = ('server_address', 'host_name')


def check_ros_env(env=os.environ) -> List[Tuple[str, str]]:
    out = []
    rmw = env.get('RMW_IMPLEMENTATION', '(default)')
    out.append((OK, f'RMW: {rmw}, ROS_DOMAIN_ID: {env.get("ROS_DOMAIN_ID", "0")}'))
    if env.get('ROS_LOCALHOST_ONLY') == '1' or \
       env.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').upper() in ('LOCALHOST', 'OFF'):
        out.append((WARN, 'ROS discovery limited to this machine: nodes on other computers '
                          '(e.g. a robot) will not see the mocap data'))
    return out


def host_of(address: str) -> str:
    """Host part of 'ip', 'ip:port' or 'name:port'."""
    host = address.rsplit(':', 1)[0] if address.count(':') == 1 else address
    return host.strip('[]')


def _interface_of(address: str) -> str:
    """Name of the local interface that owns an IPv4 address ('' if unknown)."""
    import fcntl
    import struct
    for _, name in socket.if_nameindex():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                req = struct.pack('256s', name.encode()[:15])
                got = fcntl.ioctl(s.fileno(), 0x8915, req)  # SIOCGIFADDR
            if socket.inet_ntoa(got[20:24]) == address:
                return name
        except OSError:
            continue
    return ''


def route_interface(host: str) -> Tuple[str, str]:
    """(interface, source address) used to reach host, or ('', reason)."""
    try:
        ip = socket.gethostbyname(host)
    except OSError as e:
        return '', f'cannot resolve {host}: {e}'
    try:
        out = subprocess.run(['ip', 'route', 'get', ip], capture_output=True, text=True,
                             timeout=5).stdout
        words = out.split()
        if 'dev' in words:
            src = words[words.index('src') + 1] if 'src' in words else ''
            return words[words.index('dev') + 1], src
    except (OSError, subprocess.TimeoutExpired):
        pass  # no iproute2: ask the kernel through a socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((ip, 9))  # UDP connect sends nothing; it only picks the route
            src = s.getsockname()[0]
    except OSError as e:
        return '', f'no route to {host}: {e}'
    return _interface_of(src) or 'unknown interface', src


def interface_flags(dev: str) -> int:
    try:
        with open(f'/sys/class/net/{dev}/flags') as f:
            return int(f.read().strip(), 16)
    except OSError:
        return 0


def check_server(system: str, address: str) -> List[Tuple[str, str]]:
    out = []
    host = host_of(address)
    if not host:
        return [(FAIL, f'{system}: empty server address')]
    dev, src = route_interface(host)
    if not dev:
        return [(FAIL, f'{system}: {src}')]
    if dev == 'lo':
        out.append((OK, f'{system}: server {host} is this machine'))
    else:
        out.append((OK, f'{system}: server {host} reached through {dev} ({src})'))
    flags = interface_flags(dev)
    if flags and not flags & 0x1000 and dev != 'lo':
        out.append((WARN, f'{system}: interface {dev} has no MULTICAST flag '
                          '(multicast streaming will not arrive; use unicast)'))
    try:
        if ipaddress.ip_address(socket.gethostbyname(host)).is_loopback and dev != 'lo':
            out.append((WARN, f'{system}: loopback address through {dev}'))
    except (OSError, ValueError):
        pass
    return out


def check_stats(system: str, info, stats) -> List[Tuple[str, str]]:
    label = info.state.label
    if label != 'active':
        return [(WARN if label == 'unconfigured' else OK,
                 f'{system}: {label}' + (' (configure failed? check its log)'
                                         if label == 'unconfigured' else ''))]
    if not stats:
        return [(WARN, f'{system}: active but no diagnostics yet')]
    out = []
    if stats.rate <= 0.0:
        out.append((FAIL, f'{system}: active but no frames (server streaming? network?)'))
        return out
    nominal = info.frame_rate
    if nominal > 0 and stats.rate < 0.9 * nominal:
        out.append((WARN, f'{system}: {stats.rate:.0f} Hz, below the nominal {nominal:.0f} Hz'))
    else:
        out.append((OK, f'{system}: streaming at {stats.rate:.0f} Hz'))
    expected = stats.total_frames + stats.total_dropped
    if expected and stats.total_dropped * 100 > expected:
        out.append((WARN, f'{system}: {stats.total_dropped} of {expected} frames dropped '
                          '(network load? Wi-Fi?)'))
    if stats.latency_ms > 50.0 or stats.latency_ms < -5.0:
        out.append((WARN, f'{system}: stamps {stats.latency_ms:.0f} ms off the ROS clock '
                          '(clock sync? use_sim_time mismatch?)'))
    for body, ratio in sorted(stats.tracking.items()):
        if ratio < 0.95:
            out.append((WARN, f'{system}: {body} tracked {ratio * 100:.0f}% of the frames '
                              '(occlusions, markers)'))
    return out
