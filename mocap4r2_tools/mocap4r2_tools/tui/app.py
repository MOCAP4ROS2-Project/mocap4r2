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
from textual.containers import Vertical  # noqa: E402
from textual.widgets import DataTable, Footer, Header, Input, RichLog, Static  # noqa: E402

LEVEL = {0: ('OK', 'green'), 1: ('WARN', 'yellow'), 2: ('ERROR', 'red'), 3: ('STALE', 'grey50')}
STATE_COLOR = {'active': 'green', 'inactive': 'yellow', 'unconfigured': 'grey50'}


def fmt_rate(stats, info):
    if not stats or info.state.label != 'active':
        return '-'
    nominal = f'/{info.frame_rate:.0f}' if info.frame_rate > 0 else ''
    return f'{stats.rate:.0f}{nominal} Hz'


class MocapTui(App):
    """Monitor of MOCAP4ROS2 systems."""

    TITLE = 'MOCAP4ROS2'
    CSS = """
    #systems { height: 40%; }
    #bodies { height: 1fr; }
    #log { height: 8; border-top: solid $accent; }
    #summary { height: 1; padding: 0 1; }
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

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Vertical():
            yield DataTable(id='systems', cursor_type='row', zebra_stripes=True)
            yield Static(id='summary')
            yield DataTable(id='bodies', zebra_stripes=True)
            yield Input(placeholder='session name (Enter to set, Esc to cancel)', id='session')
            yield RichLog(id='log', markup=True)
        yield Footer()

    def on_mount(self):
        systems = self.query_one('#systems', DataTable)
        systems.add_columns('System', 'Type', 'State', 'Rate', 'Latency', 'Dropped',
                            'Status', 'Session')
        bodies = self.query_one('#bodies', DataTable)
        bodies.add_columns('Body', 'Tracked', 'Tracking %', 'x', 'y', 'z', 'Error (mm)',
                           'Markers')
        self.log_line('Waiting for mocap systems on /mocap4r2/systems ...')
        self.set_interval(0.25, self.refresh_view)

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
            info = systems[name]
            st = stats.get(name)
            color = STATE_COLOR.get(info.state.label, 'white')
            level, lcolor = LEVEL.get(st.level, ('?', 'white')) if st else ('-', 'white')
            table.add_row(
                name, info.type, f'[{color}]{info.state.label}[/]', fmt_rate(st, info),
                f'{st.latency_ms:.1f} ms' if st and info.state.label == 'active' else '-',
                f'{st.total_dropped}' if st else '-',
                f'[{lcolor}]{level}[/] {st.message}' if st else '-',
                info.session_id or '-')
        self._rows = rows
        if rows:
            table.move_cursor(row=min(max(cursor, 0), len(rows) - 1))
        sel = self.selected()
        self.monitor.watch(sel)
        self.refresh_bodies(sel, stats.get(sel) if sel else None, view)

    def refresh_bodies(self, sel, st, view):
        summary = self.query_one('#summary', Static)
        bodies = self.query_one('#bodies', DataTable)
        bodies.clear()
        if not sel:
            summary.update('No system selected')
            return
        age = time.monotonic() - view.stamp if view.stamp else None
        fresh = age is not None and age < 1.0
        summary.update(
            f'[b]{sel}[/b]  markers: [green]{view.labeled}[/] labeled, '
            f'{view.unlabeled} unlabeled, [red]{view.occluded}[/] occluded'
            + ('' if fresh else '  [grey50](no data)[/]')
            + (f'   session: [b]{self.session}[/]' if self.session else ''))
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
        threading.Thread(target=work, daemon=True).start()

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
