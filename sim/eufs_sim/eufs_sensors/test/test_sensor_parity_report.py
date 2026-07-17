# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
The parity harness must recover known structure from synthetic signals.

If it cannot pull a planted noise density, correlation time or quantization
step back out of data it was fed, its verdicts about sim-vs-real are noise —
so those recoveries are pinned here.
"""
import importlib.util
import math
import os

import numpy as np
import pytest

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "sensor_parity_report.py"
)


@pytest.fixture(scope="module")
def mod():
    """Import the harness script as a module by path."""
    spec = importlib.util.spec_from_file_location(
        "sensor_parity_report", _MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_allan_recovers_white_noise_density(mod):
    """ARW from the Allan curve ~ sigma/sqrt(rate) for pure white noise."""
    rate, sigma = 200.0, 7.4e-4
    rng = np.random.default_rng(1)
    samples = rng.normal(0.0, sigma, size=int(240 * rate))
    taus, sigmas = mod.allan_deviation(samples, rate)
    assert taus, "allan curve empty"
    density = mod.arw_from_allan(taus, sigmas)
    expected = sigma / math.sqrt(rate)
    assert 0.8 * expected < density < 1.2 * expected


def test_autocorr_recovers_gm_tau(mod):
    """1/e autocorrelation time ~ the planted Gauss-Markov tau."""
    rate, tau, sigma = 100.0, 2.0, 0.03
    dt = 1.0 / rate
    alpha = math.exp(-dt / tau)
    scale = sigma * math.sqrt(1.0 - alpha * alpha)
    rng = np.random.default_rng(2)
    x = np.empty(int(240 * rate))
    x[0] = 0.0
    noise = rng.normal(0.0, scale, size=x.size)
    for i in range(1, x.size):
        x[i] = x[i - 1] * alpha + noise[i]
    measured = mod.autocorr_1e_time(x, dt)
    assert measured is not None and 1.0 < measured < 4.0


def test_wheels_section_detects_quantum(mod):
    """The detected step equals the planted AMK LSB / gear ratio."""
    quantum = 1.0 / 14.5
    rng = np.random.default_rng(3)
    true_speed = 378.16 + rng.normal(0.0, 0.5, size=4000)
    quantized = np.round(true_speed / quantum) * quantum
    stamps = list(np.arange(quantized.size) / 200.0)
    section = mod._wheels_section(stamps, list(quantized))
    assert section is not None
    assert section["quantum_rpm"] == pytest.approx(quantum, rel=0.05)
    assert section["sigma_rpm"] == pytest.approx(0.5, rel=0.3)
