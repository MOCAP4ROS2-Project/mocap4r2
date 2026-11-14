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

"""Signal processing for motion capture trajectories (no SciPy needed)."""

import math

import numpy as np


def butter2_lowpass(cutoff: float, rate: float):
    """Second order Butterworth low-pass biquad (b, a) by the bilinear transform."""
    if not 0.0 < cutoff < rate / 2.0:
        raise ValueError('cutoff must be in (0, rate / 2)')
    w = math.tan(math.pi * cutoff / rate)
    k = 1.0 / (1.0 + math.sqrt(2.0) * w + w * w)
    b = np.array([w * w * k, 2.0 * w * w * k, w * w * k])
    a = np.array([1.0, 2.0 * (w * w - 1.0) * k, (1.0 - math.sqrt(2.0) * w + w * w) * k])
    return b, a


def lfilter(b, a, x):
    """Direct form II transposed IIR filter starting at the steady state of x[0]."""
    x = np.asarray(x, dtype=float)
    y = np.empty_like(x)
    # Initial state for a constant input x[0]: no start-up transient.
    gain = b.sum() / a.sum()
    z2 = x[0] * (b[2] - a[2] * gain)
    z1 = x[0] * (b[1] - a[1] * gain) + z2
    for i, v in enumerate(x):
        out = b[0] * v + z1
        z1 = b[1] * v - a[1] * out + z2
        z2 = b[2] * v - a[2] * out
        y[i] = out
    return y


def filtfilt(cutoff: float, rate: float, x) -> np.ndarray:
    """Zero-phase low-pass (forward and backward): the usual biomechanics filter."""
    x = np.asarray(x, dtype=float)
    if len(x) < 3:
        return x.copy()
    b, a = butter2_lowpass(cutoff, rate)
    pad = min(len(x) - 1, int(3 * rate / cutoff))
    # Odd reflection at both ends reduces edge effects.
    ext = np.r_[2 * x[0] - x[pad:0:-1], x, 2 * x[-1] - x[-2:-pad - 2:-1]]
    y = lfilter(b, a, ext)
    y = lfilter(b, a, y[::-1])[::-1]
    return y[pad:pad + len(x)]


def fill_gaps(x, max_gap: int) -> np.ndarray:
    """Linear interpolation of NaN runs up to max_gap samples; longer ones stay NaN."""
    x = np.asarray(x, dtype=float).copy()
    n = len(x)
    i = 0
    while i < n:
        if not np.isnan(x[i]):
            i += 1
            continue
        j = i
        while j < n and np.isnan(x[j]):
            j += 1
        if 0 < i and j < n and j - i <= max_gap:
            x[i:j] = np.interp(np.arange(i, j), [i - 1, j], [x[i - 1], x[j]])
        i = j
    return x


def derivative(x, rate: float) -> np.ndarray:
    """Central differences (velocity from positions), same length as x."""
    x = np.asarray(x, dtype=float)
    if len(x) < 2:
        return np.zeros_like(x)
    return np.gradient(x, 1.0 / rate)
