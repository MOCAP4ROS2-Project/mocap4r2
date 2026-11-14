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

"""Terminal UI to monitor and control MOCAP4ROS2 systems (ros2 mocap4r2 tui)."""

import os
import sys
import threading
import time

# Prefer the vendored textual package.
_VENDOR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
if os.path.isdir(_VENDOR) and _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

from mocap4r2_interfaces.msg import Control  # noqa: E402
from mocap4r2_tools.client import mocap_node, MocapClient  # noqa: E402
from mocap4r2_tools.monitor import MocapMonitor  # noqa: E402
from textual.app import App, ComposeResult  # noqa: E402
from textual.binding import Binding  # noqa: E402
from textual.containers import Horizontal, Vertical  # noqa: E402
from textual.widgets import (  # noqa: E402
    Button, DataTable, Footer, Header, Input, RichLog, Static)

LEVEL = {0: ('OK', 'green'), 1: ('WARN', 'yellow'), 2: ('ERROR', 'red'), 3: ('STALE', 'grey50')}
STATE_COLOR = {'active': 'green', 'inactive': 'yellow', 'unconfigured': 'grey50',
               'finalized': 'red'}


def link_state(info, stats):
    """(text, color) of the link with the mocap software."""
    if info.state.label == 'unconfigured':
        return 'not connected', 'grey50'
    if not stats:
        return 'waiting for diagnostics', 'grey50'
    if not stats.server and not stats.connected:
        return 'unknown', 'grey50'
    if not stats.connected:
        return 'disconnected', 'red'
    if info.state.label == 'active' and stats.rate <= 0.0:
        return 'connected, no data', 'yellow'
    return 'connected', 'green'


def fmt_rate(stats, info):
    if not stats or info.state.label != 'active':
        return '-'
    nominal = f'/{info.frame_rate:.0f}' if info.frame_rate > 0 else ''
    return f'{stats.rate:.0f}{nominal} Hz'


class MocapTui(App):
    """Monitor of MOCAP4ROS2 systems."""

    TITLE = 'MOCAP4ROS2'
    CSS = """
    #top { height: 14; }
    #status-box { width: 46; border: round $accent; padding: 0 1; }
    #status { height: 1fr; }
    #toggle { width: 100%; margin-top: 1; }
    #systems { width: 1fr; }
    #summary { height: 1; padding: 0 1; }
    #bodies { height: 1fr; }
    #log { height: 7; border-top: solid $accent; }
    #session { display: none; }
    """
    BINDINGS = [
        Binding('s', 'start', 'Start'),
        Binding('x', 'stop', 'Stop'),
        Binding('a', 'start_all', 'Start all'),
        Binding('z', 'stop_all', 'Stop all'),
        Binding('n', 'session', 'Session'),
        Binding('q', 'quit', 'Quit'),
    ]

    def __init__(self, monitor: MocapMonitor):
        super().__init__()
        self.monitor = monitor
        self.session = ''
        self._rows = []
        self._busy = False

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Vertical():
            with Horizontal(id='top'):
                with Vertical(id='status-box'):
                    yield Static(id='status')
                    yield Button('Start', id='toggle', variant='success', disabled=True)
                yield DataTable(id='systems', cursor_type='row', zebra_stripes=True)
            yield Static(id='summary')
            yield DataTable(id='bodies', zebra_stripes=True)
            yield Input(placeholder='session name (Enter to set, Esc to cancel)', id='session')
            yield RichLog(id='log', markup=True)
        yield Footer()

    def on_mount(self):
        self.query_one('#status-box').border_title = 'System'
        systems = self.query_one('#systems', DataTable)
        systems.add_columns('System', 'Type', 'State', 'Link', 'Rate', 'Latency', 'Dropped',
                            'Status')
        bodies = self.query_one('#bodies', DataTable)
        bodies.add_columns('Body', 'Tracked', 'Tracking %', 'x', 'y', 'z', 'Error (mm)',
                           'Markers')
        self.log_line('Waiting for mocap systems on /mocap4r2/systems ...')
        self.set_interval(0.25, self.refresh_view)
        systems.focus()

    def log_line(self, text):
        self.query_one('#log', RichLog).write(f'[dim]{time.strftime("%H:%M:%S")}[/] {text}')

    def selected(self):
        table = self.query_one('#systems', DataTable)
        if 0 <= table.cursor_row < len(self._rows):
            return self._rows[table.cursor_row]
        return None

    def refresh_view(self):
        systems, stats, view = self.monitor.snapshot()
        table = self.query_one('#systems', DataTable)
        rows = sorted(systems)
        cursor = table.cursor_row
        table.clear()
        for name in rows:
            info, st = systems[name], stats.get(name)
            color = STATE_COLOR.get(info.state.label, 'white')
            link, lcolor = link_state(info, st)
            level, vcolor = LEVEL.get(st.level, ('?', 'white')) if st else ('-', 'white')
            table.add_row(
                name, info.type, f'[{color}]{info.state.label}[/]', f'[{lcolor}]{link}[/]',
                fmt_rate(st, info),
                f'{st.latency_ms:.1f} ms' if st and info.state.label == 'active' else '-',
                f'{st.total_dropped}' if st else '-',
                f'[{vcolor}]{level}[/] {st.message}' if st else '-')
        self._rows = rows
        if rows:
            table.move_cursor(row=min(max(cursor, 0), len(rows) - 1))
        sel = self.selected()
        self.monitor.watch(sel)
        info = systems.get(sel) if sel else None
        self.refresh_status(sel, info, stats.get(sel) if sel else None)
        self.refresh_bodies(sel, stats.get(sel) if sel else None, view)

    def refresh_status(self, sel, info, st):
        status = self.query_one('#status', Static)
        button = self.query_one('#toggle', Button)
        if not info:
            status.update('[grey50]No mocap system selected.\n\n'
                          'Start a driver, e.g.\nros2 launch mocap4r2_sim_driver '
                          'sim_driver.launch.py[/]')
            button.disabled = True
            return
        state = info.state.label
        color = STATE_COLOR.get(state, 'white')
        link, lcolor = link_state(info, st)
        server = st.server if st and st.server else '-'
        rate = fmt_rate(st, info)
        lines = [
            f'[b]{sel}[/b] ({info.type})',
            f'Lifecycle  [{color}][b]{state.upper()}[/b][/]',
            f'Software   [{lcolor}]{link}[/]',
            f'           [dim]{server}[/]',
            f'Rate       {rate}   latency '
            + (f'{st.latency_ms:.1f} ms' if st and state == 'active' else '-'),
            f'Dropped    {st.total_dropped if st else "-"}   frame [dim]{info.frame_id}[/]',
            f'Session    {info.session_id or "-"}',
        ]
        status.update('\n'.join(lines))
        active = state == 'active'
        button.label = 'Stop (deactivate)' if active else 'Start (activate)'
        button.variant = 'error' if active else 'success'
        button.disabled = self._busy or state == 'finalized'

    def refresh_bodies(self, sel, st, view):
        summary = self.query_one('#summary', Static)
        bodies = self.query_one('#bodies', DataTable)
        bodies.clear()
        if not sel:
            summary.update('')
            return
        age = time.monotonic() - view.stamp if view.stamp else None
        fresh = age is not None and age < 1.0
        summary.update(
            f'markers: [green]{view.labeled}[/] labeled, {view.unlabeled} unlabeled, '
            f'[red]{view.occluded}[/] occluded' + ('' if fresh else '  [grey50](no data)[/]')
            + (f'   next session: [b]{self.session}[/]' if self.session else ''))
        if not view.bodies:
            return
        for rb in view.bodies.rigid_bodies:
            p = rb.pose.position
            ratio = st.tracking.get(rb.name) if st else None
            ok = '[green]yes[/]' if rb.tracking_valid else '[red]LOST[/]'
            bodies.add_row(
                rb.name, ok, f'{ratio * 100:.0f}' if ratio is not None else '-',
                f'{p.x:+.3f}', f'{p.y:+.3f}', f'{p.z:+.3f}',
                f'{rb.mean_error * 1000:.2f}' if rb.mean_error >= 0 else '-',
                str(len(rb.markers)))

    def control(self, command, targets):
        verb = 'START' if command == Control.START else 'STOP'
        what = ', '.join(targets) if targets else 'all systems'
        self.log_line(f'{verb} {what}' + (f' (session {self.session})' if self.session else ''))
        session = self.session
        self._busy = True

        def work():
            try:
                with mocap_node('mocap4r2_tui_control') as node:
                    result = MocapClient(node).control(command, targets, session_id=session)
                lines = [f'[green]{n}: ok[/]' if a.success else f'[red]{n}: {a.message}[/]'
                         for n, a in result.acks.items()]
                lines += [f'[red]{n}: no answer[/]' for n in result.missing]
            except Exception as e:  # noqa: B902
                lines = [f'[red]{e}[/]']
            for line in lines:
                self.call_from_thread(self.log_line, line)
            self._busy = False
        threading.Thread(target=work, daemon=True).start()

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id != 'toggle' or not self.selected():
            return
        systems, _, _ = self.monitor.snapshot()
        info = systems.get(self.selected())
        if info:
            command = Control.STOP if info.state.label == 'active' else Control.START
            self.control(command, [self.selected()])

    def action_start(self):
        if self.selected():
            self.control(Control.START, [self.selected()])

    def action_stop(self):
        if self.selected():
            self.control(Control.STOP, [self.selected()])

    def action_start_all(self):
        self.control(Control.START, [])

    def action_stop_all(self):
        self.control(Control.STOP, [])

    def action_session(self):
        box = self.query_one('#session', Input)
        box.display = True
        box.value = self.session
        box.focus()

    def on_input_submitted(self, event: Input.Submitted):
        self.session = event.value.strip()
        event.input.display = False
        self.query_one('#systems', DataTable).focus()
        self.log_line(f'session set to "{self.session}"' if self.session else 'session cleared')

    def on_key(self, event):
        box = self.query_one('#session', Input)
        if event.key == 'escape' and box.display:
            box.display = False
            self.query_one('#systems', DataTable).focus()


def run_app():
    monitor = MocapMonitor('mocap4r2_tui')
    try:
        MocapTui(monitor).run()
    finally:
        monitor.close()


if __name__ == '__main__':
    run_app()
