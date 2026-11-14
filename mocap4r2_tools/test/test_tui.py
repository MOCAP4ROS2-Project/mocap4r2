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

"""Smoke test of the TUI against simulated systems."""

import asyncio
import os
import random
import signal
import subprocess
import time

import pytest


@pytest.fixture(scope='module')
def sims():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(201, 230))
    procs = [subprocess.Popen(
        ['ros2', 'run', 'mocap4r2_sim_driver', 'sim_driver', '--ros-args', '-r',
         '__node:=tui_sim', '-p', 'rate:=60.0', '-p', 'autostart:=activate'],
        start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)]
    yield procs
    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def test_tui_shows_systems_and_bodies(sims):
    from mocap4r2_tools.monitor import MocapMonitor
    try:
        from mocap4r2_tools.tui.app import MocapTui
    except ImportError as e:  # Humble: system Python packages too old for Textual
        pytest.skip(f'Textual not usable here: {e}')
    from textual.widgets import DataTable, RichLog

    monitor = MocapMonitor('tui_test')

    async def run():
        app = MocapTui(monitor)
        async with app.run_test(size=(140, 40)) as pilot:
            systems = app.query_one('#systems', DataTable)
            bodies = app.query_one('#bodies', DataTable)
            end = time.monotonic() + 20
            while time.monotonic() < end and (systems.row_count == 0 or bodies.row_count < 2):
                await pilot.pause(0.25)
            assert systems.row_count >= 1
            assert bodies.row_count == 2  # robot and wand of the simulated system
            row = systems.get_row_at(0)
            assert row[0] == '/tui_sim'
            assert 'active' in str(row[2])
            # The status box shows lifecycle and link with the (simulated) software.
            end = time.monotonic() + 10
            while time.monotonic() < end and \
                    'simulated scene' not in str(app.query_one('#status').render()):
                await pilot.pause(0.25)
            status = str(app.query_one('#status').render())
            assert 'ACTIVE' in status and 'connected' in status and 'simulated scene' in status
            # Stop it with the button of the status box and see the state change.
            from textual.widgets import Button
            button = app.query_one('#toggle', Button)
            assert 'Stop' in str(button.label)
            await pilot.click('#toggle')
            end = time.monotonic() + 15
            while time.monotonic() < end and 'inactive' not in str(systems.get_row_at(0)[2]):
                await pilot.pause(0.25)
            assert 'inactive' in str(systems.get_row_at(0)[2])
            log = app.query_one('#log', RichLog)
            assert len(log.lines) >= 2
            await pilot.press('q')

    try:
        asyncio.run(run())
    finally:
        monitor.close()
