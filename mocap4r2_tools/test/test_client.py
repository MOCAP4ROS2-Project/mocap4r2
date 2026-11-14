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

"""End-to-end tests of the control client against simulated systems."""

import os
import random
import signal
import subprocess
import time

from lifecycle_msgs.msg import State
from mocap4r2_interfaces.msg import Control
from mocap4r2_tools.client import mocap_node, MocapClient, MocapError
import pytest


def _launch_sim(name, namespace='', extra=()):
    cmd = ['ros2', 'run', 'mocap4r2_sim_driver', 'sim_driver', '--ros-args',
           '-r', f'__node:={name}', '-p', 'rate:=50.0', *extra]
    if namespace:
        cmd += ['-r', f'__ns:={namespace}']
    return subprocess.Popen(cmd, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


@pytest.fixture(scope='module', autouse=True)
def isolated_domain():
    # Keep these tests away from other ROS processes on the machine.
    old = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
    yield
    if old is None:
        del os.environ['ROS_DOMAIN_ID']
    else:
        os.environ['ROS_DOMAIN_ID'] = old


@pytest.fixture(scope='module')
def sims(isolated_domain):
    procs = [_launch_sim('sim_a'), _launch_sim('sim_b', '/r1')]
    yield procs
    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def _wait_systems(client, names, timeout=20.0):
    end = time.monotonic() + timeout
    systems = {}
    while time.monotonic() < end:
        systems = client.systems(1.0)
        if all(n in systems for n in names):
            return systems
    return systems


def _state(client, name):
    return client.systems(1.0)[name].state.id


def test_discovery(sims):
    with mocap_node('test') as node:
        client = MocapClient(node)
        systems = _wait_systems(client, ['/sim_a', '/r1/sim_b'])
        assert '/sim_a' in systems and '/r1/sim_b' in systems
        a = systems['/sim_a']
        assert a.type == 'sim'
        assert a.frame_id == 'mocap'
        assert a.frame_rate == pytest.approx(50.0)
        assert a.state.id == State.PRIMARY_STATE_INACTIVE
        assert '/rigid_bodies' in a.topics
        assert 'mocap4r2_interfaces/msg/RigidBodies' in a.topic_types
        assert '/r1/rigid_bodies' in systems['/r1/sim_b'].topics


def test_resolve_names(sims):
    with mocap_node('test') as node:
        client = MocapClient(node)
        systems = _wait_systems(client, ['/sim_a', '/r1/sim_b'])
        resolved = client.resolve(['sim_a', '/r1/sim_b', 'sim_b'], systems)
        assert resolved == ['/sim_a', '/r1/sim_b', '/r1/sim_b']
        with pytest.raises(MocapError, match='unknown'):
            client.resolve(['nope'], systems)
        dup = dict(systems)
        dup['/r2/sim_a'] = systems['/sim_a']
        with pytest.raises(MocapError, match='ambiguous'):
            client.resolve(['sim_a'], dup)


def test_start_stop_one_and_all(sims):
    with mocap_node('test') as node:
        client = MocapClient(node)
        _wait_systems(client, ['/sim_a', '/r1/sim_b'])

        r = client.control(Control.START, ['/r1/sim_b'], session_id='trial1')
        assert r.ok, (r.acks, r.missing)
        assert list(r.acks) == ['/r1/sim_b']
        assert r.acks['/r1/sim_b'].session_id == 'trial1'
        assert client.spin_until(
            lambda: _state(client, '/r1/sim_b') == State.PRIMARY_STATE_ACTIVE, 5.0)
        assert _state(client, '/sim_a') == State.PRIMARY_STATE_INACTIVE
        assert client.systems(1.0)['/r1/sim_b'].session_id == 'trial1'

        # All systems: the active one answers too (idempotent).
        r = client.control(Control.START, [], session_id='trial2')
        assert r.ok and set(r.acks) >= {'/sim_a', '/r1/sim_b'}

        r = client.control(Control.STOP, [])
        assert r.ok
        assert client.spin_until(
            lambda: _state(client, '/sim_a') == State.PRIMARY_STATE_INACTIVE and
            _state(client, '/r1/sim_b') == State.PRIMARY_STATE_INACTIVE, 5.0)


def test_unknown_target_reports_missing(sims):
    with mocap_node('test') as node:
        client = MocapClient(node)
        systems = _wait_systems(client, ['/sim_a'])
        r = client.control(Control.START, ['/ghost'], timeout=1.0, systems=systems)
        assert not r.ok
        assert r.missing == ['/ghost']


def test_failed_start_is_reported(isolated_domain):
    # A system that cannot configure (invalid rate) answers START with a failure.
    bad = _launch_sim('sim_bad', extra=['-p', 'rate:=-1.0', '-p', 'autostart:=none'])
    try:
        with mocap_node('test') as node:
            client = MocapClient(node)
            _wait_systems(client, ['/sim_bad'])
            r = client.control(Control.START, ['/sim_bad'])
            assert not r.ok
            assert not r.acks['/sim_bad'].success
            assert 'unconfigured' in r.acks['/sim_bad'].message
    finally:
        os.killpg(bad.pid, signal.SIGINT)
        bad.wait(timeout=10)


def test_cli_verbs(sims):
    env = dict(os.environ)
    out = subprocess.run(['ros2', 'mocap4r2', 'list'], capture_output=True, text=True,
                         env=env, timeout=60)
    assert out.returncode == 0, out.stderr
    assert '/sim_a' in out.stdout and '/r1/sim_b' in out.stdout

    out = subprocess.run(['ros2', 'mocap4r2', 'info', 'sim_b'], capture_output=True, text=True,
                         env=env, timeout=60)
    assert out.returncode == 0, out.stderr
    assert '/r1/rigid_bodies' in out.stdout

    out = subprocess.run(['ros2', 'mocap4r2', 'start', 'sim_a', '--session', 's9'],
                         capture_output=True, text=True, env=env, timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert '/sim_a: started' in out.stdout

    out = subprocess.run(['ros2', 'mocap4r2', 'stop', 'sim_a'],
                         capture_output=True, text=True, env=env, timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert '/sim_a: stopped' in out.stdout

    out = subprocess.run(['ros2', 'mocap4r2', 'start', 'ghost'],
                         capture_output=True, text=True, env=env, timeout=60)
    assert out.returncode != 0
    assert 'unknown mocap system' in out.stdout + out.stderr
