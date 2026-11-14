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

"""Unit tests of the pure parts of the tools (no ROS graph needed)."""

import datetime

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from mocap4r2_interfaces.msg import MocapSystemInfo, RigidBodies, RigidBody
from mocap4r2_tools import doctor
from mocap4r2_tools.monitor import parse_diagnostic, SystemStats
from mocap4r2_tools.verb.bodies import bodies_table
from mocap4r2_tools.verb.record import session_metadata


def status(values, level=0, message='streaming'):
    s = DiagnosticStatus()
    s.level = bytes([level])
    s.message = message
    s.values = [KeyValue(key=k, value=v) for k, v in values.items()]
    return s


def info(state='active', rate=100.0):
    i = MocapSystemInfo()
    i.system = '/sim'
    i.type = 'sim'
    i.frame_id = 'mocap'
    i.frame_rate = rate
    i.state.label = state
    i.topics = ['/rigid_bodies']
    i.topic_types = ['mocap4r2_interfaces/msg/RigidBodies']
    return i


def test_parse_diagnostic():
    name, s = parse_diagnostic(status({
        'system': '/r1/opti', 'rate_hz': '119.5', 'latency_mean_ms': '4.2', 'dropped': '1',
        'total_dropped': '3', 'total_frames': '1000', 'tracking/robot': '0.98',
        'tracking/bad': 'x'}, level=1, message='dropping frames'))
    assert name == '/r1/opti'
    assert s.level == 1 and s.message == 'dropping frames'
    assert s.rate == 119.5 and s.latency_ms == 4.2
    assert s.total_dropped == 3 and s.total_frames == 1000
    assert s.tracking == {'robot': 0.98}
    # Diagnostics of other nodes are ignored; malformed numbers become 0.
    assert parse_diagnostic(status({'rate_hz': '1'})) is None
    _, s = parse_diagnostic(status({'system': '/a', 'rate_hz': 'nan?'}))
    assert s.rate == 0.0


def test_check_stats_cases():
    assert doctor.check_stats('/s', info('inactive'), None)[0][0] == doctor.OK
    assert doctor.check_stats('/s', info('unconfigured'), None)[0][0] == doctor.WARN
    assert doctor.check_stats('/s', info(), None)[0][0] == doctor.WARN
    assert doctor.check_stats('/s', info(), SystemStats(rate=0.0))[0][0] == doctor.FAIL
    good = doctor.check_stats('/s', info(), SystemStats(rate=99.0, total_frames=1000))
    assert [lvl for lvl, _ in good] == [doctor.OK]
    slow = doctor.check_stats('/s', info(), SystemStats(rate=50.0))
    assert slow[0][0] == doctor.WARN
    drops = doctor.check_stats('/s', info(), SystemStats(rate=100, total_frames=900,
                                                         total_dropped=100))
    assert any('dropped' in t for _, t in drops)
    skew = doctor.check_stats('/s', info(), SystemStats(rate=100, latency_ms=900.0))
    assert any('clock' in t for _, t in skew)
    lost = doctor.check_stats('/s', info(), SystemStats(rate=100, tracking={'wand': 0.5}))
    assert any('wand' in t for _, t in lost)


def test_ros_env_and_hosts():
    assert len(doctor.check_ros_env({})) == 1
    assert doctor.check_ros_env({'ROS_LOCALHOST_ONLY': '1'})[1][0] == doctor.WARN
    assert doctor.check_ros_env({'ROS_AUTOMATIC_DISCOVERY_RANGE': 'localhost'})[1][0] == \
        doctor.WARN
    assert doctor.host_of('192.168.10.1:801') == '192.168.10.1'
    assert doctor.host_of('10.0.0.2') == '10.0.0.2'
    assert doctor.host_of('vicon-pc:801') == 'vicon-pc'


def test_check_server_local_and_unresolvable():
    local = doctor.check_server('/s', '127.0.0.1')
    assert local[0][0] == doctor.OK
    bad = doctor.check_server('/s', 'no-such-host.invalid:801')
    assert bad[0][0] == doctor.FAIL
    assert doctor.check_server('/s', '')[0][0] == doctor.FAIL


def test_bodies_table():
    msg = RigidBodies()
    rb = RigidBody()
    rb.name = 'robot'
    rb.tracking_valid = True
    rb.pose.position.x = 1.23456
    rb.pose.orientation.w = 1.0
    rb.mean_error = 0.0005
    lost = RigidBody()
    lost.name = 'wand'
    lost.mean_error = -1.0
    msg.rigid_bodies = [rb, lost]
    table = bodies_table(msg).splitlines()
    assert table[0].startswith('BODY')
    assert 'robot' in table[1] and '+1.2346' in table[1] and '0.50' in table[1]
    assert 'LOST' in table[2] and table[2].rstrip().endswith('0')


def test_session_metadata():
    start = datetime.datetime(2026, 11, 14, 10, 0, 0)
    end = start + datetime.timedelta(seconds=12)
    meta = session_metadata('trial1', [info()], start, end, '/tmp/x/trial1')['mocap4r2_session']
    assert meta['session'] == 'trial1'
    assert meta['bag'] == 'trial1'
    assert meta['systems'][0]['topics'][0] == {
        'name': '/rigid_bodies', 'type': 'mocap4r2_interfaces/msg/RigidBodies'}
    assert meta['end'] == '2026-11-14T10:00:12'
