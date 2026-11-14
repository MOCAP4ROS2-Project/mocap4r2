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

from mocap4r2_tools.verb import VerbExtension


class TuiVerb(VerbExtension):
    """Terminal UI to monitor and control the mocap systems."""

    def main(self, *, args):
        try:
            from mocap4r2_tools.tui.app import run_app
        except ImportError as e:
            # Ubuntu 22.04 (Humble) ships too old rich/typing_extensions for Textual.
            return (f'Error: the terminal UI needs newer Python packages ({e}).\n'
                    "Install them with: pip install --user 'textual>=6'")
        run_app()
        return 0
