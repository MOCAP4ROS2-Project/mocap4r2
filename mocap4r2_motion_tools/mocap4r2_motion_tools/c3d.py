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

"""
Minimal C3D writer (3D points, floating point, Intel byte order).

C3D is the exchange format of biomechanics (Visual3D, OpenSim, Mokka, ezc3d).
Points are written in millimetres, the usual C3D unit.
"""

import struct
from typing import Sequence

import numpy as np

BLOCK = 512
INTEL = 84


def _group(gid: int, name: str, desc: str = '') -> bytes:
    n = name.upper().encode()
    d = desc.encode()
    body = struct.pack('<h', 2 + 1 + len(d)) + bytes([len(d)]) + d
    return struct.pack('<bb', len(n), -gid) + n + body


def _param(gid: int, name: str, kind: int, dims: Sequence[int], data: bytes,
           desc: str = '') -> bytes:
    n = name.upper().encode()
    d = desc.encode()
    rest = struct.pack('<bB', kind, len(dims)) + bytes(dims) + data + bytes([len(d)]) + d
    return struct.pack('<bb', len(n), gid) + n + struct.pack('<h', 2 + len(rest)) + rest


def _int16(gid, name, value):
    return _param(gid, name, 2, [], struct.pack('<h', value))


def _float(gid, name, value):
    return _param(gid, name, 4, [], struct.pack('<f', value))


def _chars(gid, name, strings: Sequence[str], width: int = 0):
    width = width or max([len(s) for s in strings] + [1])
    data = b''.join(s.encode()[:width].ljust(width) for s in strings)
    if len(strings) == 1:
        return _param(gid, name, -1, [width], data)
    return _param(gid, name, -1, [width, len(strings)], data)


def write_c3d(path: str, labels: Sequence[str], points: np.ndarray, rate: float,
              valid: np.ndarray = None):
    """
    Write points (frames x points x 3, metres) with their labels at rate Hz.

    valid (frames x points, bool) marks samples seen by the system; NaN points
    are also written as invalid.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 3 or points.shape[2] != 3 or points.shape[1] != len(labels):
        raise ValueError('points must be frames x len(labels) x 3')
    if rate <= 0:
        raise ValueError('rate must be > 0')
    n_frames, n_points = points.shape[0], points.shape[1]
    if n_points > 32767:
        raise ValueError('too many points for C3D')
    if valid is None:
        valid = np.ones((n_frames, n_points), dtype=bool)
    valid = valid & ~np.isnan(points).any(axis=2)

    params = b''.join([
        _group(1, 'POINT', '3D points'),
        _int16(1, 'USED', n_points),
        _int16(1, 'FRAMES', min(n_frames, 32767)),
        _int16(1, 'DATA_START', 0),  # patched below
        _float(1, 'SCALE', -1.0),
        _float(1, 'RATE', float(rate)),
        _chars(1, 'LABELS', list(labels) or [''], 30),
        _chars(1, 'UNITS', ['mm']),
        _group(2, 'ANALOG', 'analog channels'),
        _int16(2, 'USED', 0),
        _float(2, 'RATE', float(rate)),
        _group(3, 'TRIAL', 'trial'),
        # 32-bit first and last frame (two words each), for more than 32767 frames
        _param(3, 'ACTUAL_START_FIELD', 2, [2], struct.pack('<HH', 1, 0)),
        _param(3, 'ACTUAL_END_FIELD', 2, [2],
               struct.pack('<HH', n_frames & 0xffff, n_frames >> 16)),
        b'\x00\x00\x00\x00',  # end of parameters
    ])
    param_blocks = (4 + len(params) + BLOCK - 1) // BLOCK
    data_start = 2 + param_blocks
    marker = struct.pack('<bb', len('DATA_START'), 1) + b'DATA_START'
    at = params.index(marker) + len(marker) + 2 + 2  # offset, type, dims
    params = params[:at] + struct.pack('<h', data_start) + params[at + 2:]

    header = bytearray(BLOCK)
    struct.pack_into('<BB', header, 0, 2, 0x50)
    struct.pack_into('<hhhh', header, 2, n_points, 0, 1, min(n_frames, 65535))
    struct.pack_into('<hfhhf', header, 10, 10, -1.0, data_start, 0, float(rate))

    section = bytearray(param_blocks * BLOCK)
    struct.pack_into('<BBBB', section, 0, 1, 0x50, param_blocks, INTEL)
    section[4:4 + len(params)] = params

    mm = np.where(valid[..., None], points * 1000.0, 0.0).astype('<f4')
    residual = np.where(valid, 0.0, -1.0).astype('<f4')[..., None]
    data = np.concatenate([mm, residual], axis=2).tobytes()
    data += bytes((-len(data)) % BLOCK)

    with open(path, 'wb') as f:
        f.write(header)
        f.write(section)
        f.write(data)
