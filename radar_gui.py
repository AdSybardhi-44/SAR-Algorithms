# radar_gui.py

"""
Graphical user interface for the X1000 radar.

Workflow:

    1. Set the controllable radar parameters.
    2. Answer whether the empty-scene background is removed.
    3. Answer whether the antenna coupling at near range is high-pass
       filtered (Butterworth, zero phase with scipy.signal.filtfilt).
    4. Answer whether the RX traces are combined coherently (RX
       beamforming, calibrated on a corner reflector at run start).
    5. Choose whether the acquisition is saved, displayed in real
       time, or both, then press Start.

The real-time display shows absolute levels in dBFS, as in the 2piSENSE
examples: 20*log10(2*|rfft(x*w)/sum(w)| / full_scale), so a full-scale
sine reads 0 dBFS. With RX beamforming, the top panel shows the
beamformed profile, a weighted coherent sum of the traces (see
Beamformer), so its target peaks are higher than in a single trace.

Run from the repository folder:

    .venv\\Scripts\\python.exe radar_gui.py

Saved files (.npz or .mat) contain:

    rx0, rx1, rx2                 raw IF samples of the enabled traces,
                                  shape (n_frames, sweep_count, sweep_points)
    frequency_hz, range_axis_m    delivered frequency axis, one-way range axis
    frame_timestamps_s            PC time (POSIX s, UTC) of every frame
    trigger_timestamps_ns         radar-clock trigger time of every frame
                                  (-1 = n/a); the radar clock is not
                                  synchronized to the PC, use it for
                                  frame-to-frame timing only
    config_*                      applied configuration (RadarConfig)
    setup_report_json             radar_health.json written by setup_radar
    device_info_*_json            device information (temperature, ...)
                                  before and after the recording

When the background is removed, additionally:

    rxN_background                averaged background, shape (sweep_points,)
    rxN_background_raw            background acquisitions
    rxN_background_removed        rxN - rxN_background (float32)

When the high-pass filter is on, additionally:

    rxN_highpass                  rxN (minus rxN_background when the
                                  background is removed), filtered along
                                  each sweep with filtfilt (float32)
    highpass_b, highpass_a        Butterworth coefficients
    highpass_cutoff_m, highpass_order, highpass_padlen, highpass_method,
    highpass_cutoff_if_hz, highpass_cutoff_normalized

When RX beamforming is on, additionally:

    rx_beamformed                 beamformed complex IF (complex64), shape
                                  (n_frames, sweep_count, sweep_points);
                                  range profile = fft(rx_beamformed * w)
                                  [..., :sweep_points // 2 + 1] / sum(w)
    beamforming_weights, beamforming_shift_bins, beamforming_traces,
    beamforming_reference_trace, beamforming_method, and the
    calibration results beamforming_target_range_m,
    beamforming_channel_snr_db, beamforming_phase_std_deg,
    beamforming_snr_gain_db
"""

import json
import math
import queue
import tempfile
import threading
import time
import tkinter as tk
import traceback
from collections import deque
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import numpy as np
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)
from matplotlib.figure import Figure
from scipy.signal import butter, filtfilt, freqz, sosfiltfilt

from radar_acquisition import (
    AcquisitionCancelled,
    InvalidFrameError,
    acquire_background,
    acquire_frame,
)
from radar_setup import setup_radar
from radar_types import (
    RadarBackground,
    RadarConfig,
    RadarFrame,
    RadarMeasurement,
)


C = 299_792_458.0

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "Results"
HEALTH_REPORT_FILE = SCRIPT_DIR / "radar_health.json"

# Same processing as real_time_detection_dud.py
KAISER_BETA = 2.5

# filtfilt pads each sweep until the filter impulse response has decayed
# to this level, so its start-up transient never reaches the data.
HIGHPASS_PADDING_DECAY = 1e-6
# (b, a) coefficients lose accuracy for high orders and low cutoffs;
# filtfilt must match the second-order-section result to this level.
HIGHPASS_MAX_RELATIVE_ERROR = 1e-5
HIGHPASS_MAX_ORDER = 10
HIGHPASS_FRAMES_PER_CHUNK = 256

# RX beamforming: per-trace peak search for the calibration target.
BEAMFORMING_ZERO_PADDING = 16
BEAMFORMING_PEAK_SEARCH_BINS = 3
# Range excluded around the calibration target in the noise estimate.
BEAMFORMING_TARGET_EXCLUSION_M = 0.05
BEAMFORMING_MIN_CALIBRATION_FRAMES = 10
BEAMFORMING_MIN_SNR_DB = 15.0
BEAMFORMING_MAX_PHASE_STD_DEG = 10.0
PROCESSING_CHUNK_BYTES = 64 * 1024**2

POLL_INTERVAL_MS = 20
MAX_EVENTS_PER_POLL = 200
MAX_LOG_LINES = 2000
RATE_HISTORY_LENGTH = 20
PROGRESS_INTERVAL_S = 0.1
MAX_CONSECUTIVE_INVALID_FRAMES = 5
LARGE_RECORDING_BYTES = 2 * 1024**3

TRACE_LABELS = {
    0: "0 (TRx)",
    1: "1 (Rx)",
    2: "2 (Rx)",
}

DEFAULT_VALUES = {
    "f_start_ghz": "126",
    "f_stop_ghz": "182",
    "sweep_time_ms": "1",
    "sweep_period_ms": "2",
    "sweep_count": "1",
    "background_acquisitions": "100",
    # Measured on this radar: its own near-range clutter reaches ~20 cm.
    "highpass_cutoff_m": "0.2",
    "highpass_order": "6",
    "bf_calibration_frames": "50",
    # Starts beyond the strongest near-range clutter of this radar.
    "bf_search_min_m": "0.2",
    "bf_search_max_m": "6",
    "frames_to_record": "100",
    "plot_min_range_m": "0",
    "plot_max_range_m": "6",
    # Absolute levels in dBFS; measured on this radar: noise floor about
    # -104 to -110 dBFS, near-range coupling up to about -52 dBFS.
    "plot_min_db": "-130",
    "plot_max_db": "-30",
    "peak_min_range_m": "0.1",
    "peak_max_range_m": "6",
}


# =====================================================================
# Data containers
# =====================================================================

@dataclass(frozen=True)
class RunSettings:

    f_start_hz: float
    f_stop_hz: float
    sweep_time_s: float
    sweep_period_s: float
    sweep_count: int
    traces: tuple

    remove_background: bool
    background_acquisitions: int

    highpass: bool
    highpass_cutoff_m: float
    highpass_order: int

    beamforming: bool
    bf_calibration_frames: int
    bf_search_min_m: float
    bf_search_max_m: float

    save: bool
    display: bool

    frames_to_record: int
    output_dir: Path
    file_format: str

    plot_min_range_m: float
    plot_max_range_m: float
    plot_min_db: float
    plot_max_db: float
    peak_min_range_m: float
    peak_max_range_m: float


@dataclass(frozen=True)
class HighpassFilter:
    """
    Butterworth high-pass along each sweep, applied with filtfilt
    (zero phase, so the phase of the range profile is preserved).

    Range maps to IF frequency, so a cutoff range R_c gives the
    normalized cutoff (1 = Nyquist = maximum range):

        Wn = R_c / R_max = 4 * R_c * |delta_f| / c
    """

    cutoff_m: float
    order: int
    max_range_m: float
    cutoff_normalized: float
    cutoff_if_hz: float
    b: np.ndarray
    a: np.ndarray
    padlen: int

    def apply(self, if_data):
        """Filter along the last axis (fast time)."""

        return filtfilt(
            self.b,
            self.a,
            if_data,
            axis=-1,
            padtype="odd",
            padlen=self.padlen,
        )

    def gain_db(self, range_m):
        """Gain of the forward-backward filter at range_m."""

        _, response = freqz(
            self.b,
            self.a,
            worN=[math.pi * range_m / self.max_range_m],
        )

        return 40.0 * math.log10(max(abs(response[0]), 1e-300))

    def describe(self):

        c = self.cutoff_m

        return (
            f"High-pass filter: Butterworth order {self.order}, cutoff "
            f"{c:g} m (IF {self.cutoff_if_hz / 1e3:.2f} kHz), zero-phase "
            f"filtfilt, {self.padlen} samples odd padding.\n"
            f"  Gain: {self.gain_db(c / 2):.1f} dB at {c / 2:g} m, "
            f"{self.gain_db(c):.1f} dB at {c:g} m, "
            f"{self.gain_db(2 * c):.2f} dB at {2 * c:g} m"
        )


@dataclass(frozen=True)
class Beamformer:
    """
    Calibrated RX beamformer: delay-and-sum with maximum-ratio weights.

    Each trace k is shifted in range by shift_bins[k], so the calibration
    target lies at the range it has in the reference trace, rotated to
    the reference phase and weighted by |h_k| / noise_k (maximum-ratio
    combining, the SNR-optimal coherent combination for independent
    noise). The amplitude weights sum to the number of traces, so equal
    traces give their plain coherent sum:

        z[n] = sum_k weights[k] * x_k[n] * exp(-2j*pi*shift_bins[k]*n/N)
    """

    trace_map: tuple
    reference_trace: int
    shift_bins: np.ndarray
    weights: np.ndarray
    # weights[k] * exp(-2j*pi*shift_bins[k]*n/N), shape (n_traces, N)
    steering: np.ndarray
    range_bin_m: float
    target_range_m: float
    target_dbfs: np.ndarray
    noise_dbfs: np.ndarray
    channel_snr_db: np.ndarray
    phase_std_deg: np.ndarray
    combined_snr_db: float
    n_frames: int

    @property
    def snr_gain_db(self):
        """Measured combined SNR minus that of the best single trace."""

        return self.combined_snr_db - float(np.max(self.channel_snr_db))

    def combine(self, if_data):
        """(..., n_traces, sweep_points) real IF -> (..., sweep_points) complex."""

        return np.einsum("...kn,kn->...n", if_data, self.steering)

    def problems(self):
        """Reasons why the calibration may be unusable (empty if fine)."""

        problems = []
        best_snr_db = float(np.max(self.channel_snr_db))

        if best_snr_db < BEAMFORMING_MIN_SNR_DB:
            problems.append(
                f"The calibration target is weak: the best trace has "
                f"{best_snr_db:.1f} dB SNR (at least "
                f"{BEAMFORMING_MIN_SNR_DB:g} dB recommended)."
            )

        for trace, snr_db, phase_std in zip(
            self.trace_map, self.channel_snr_db, self.phase_std_deg,
        ):
            if snr_db >= 10.0 and phase_std > BEAMFORMING_MAX_PHASE_STD_DEG:
                problems.append(
                    f"Trace {trace}: the target phase varied by "
                    f"{phase_std:.1f} deg between frames (did the target "
                    f"or the radar move?)."
                )

        if self.snr_gain_db < 0.0:
            problems.append(
                f"The combined SNR is {-self.snr_gain_db:.1f} dB lower "
                f"than that of the best single trace."
            )

        return problems

    def describe(self):

        best = int(np.argmax(self.channel_snr_db))

        lines = [
            f"Beamforming calibrated on the target at "
            f"{self.target_range_m:.4f} m (range of trace "
            f"{self.reference_trace}), {self.n_frames} frames:"
        ]

        for k, trace in enumerate(self.trace_map):
            lines.append(
                f"  Trace {trace}: target {self.target_dbfs[k]:.1f} dBFS, "
                f"noise {self.noise_dbfs[k]:.1f} dBFS/bin, SNR "
                f"{self.channel_snr_db[k]:.1f} dB, shift "
                f"{self.shift_bins[k]:+.3f} bins "
                f"({self.shift_bins[k] * self.range_bin_m * 1e3:+.2f} mm), "
                f"phase std {self.phase_std_deg[k]:.1f} deg, weight "
                f"{abs(self.weights[k]):.2f} at "
                f"{math.degrees(np.angle(self.weights[k])):+.1f} deg"
            )

        lines.append(
            f"  Combined SNR {self.combined_snr_db:.1f} dB = "
            f"{self.snr_gain_db:+.1f} dB over the best single trace "
            f"(trace {self.trace_map[best]})"
        )

        return "\n".join(lines)


@dataclass
class DisplaySetup:

    run_id: int
    trace_map: list
    range_axis_m: np.ndarray
    background_if: np.ndarray | None
    highpass: HighpassFilter | None
    beamformer: Beamformer | None
    sweep_points: int
    full_scale: float
    settings: RunSettings


@dataclass
class DisplayFrame:

    run_id: int
    frame: RadarFrame
    frame_number: int
    rate_hz: float
    acquisition_ms: float


@dataclass
class TraceArtists:

    line: object
    marker: object
    label: object
    legend: object


# =====================================================================
# Processing
# =====================================================================

def frequency_step(frequency_hz):
    """Mean step of a frequency axis (Hz, positive)."""

    step_hz = abs(float(np.mean(np.diff(frequency_hz))))

    if not math.isfinite(step_hz) or step_hz <= 0.0:
        raise ValueError(
            "The frequency axis has no valid frequency step."
        )

    return step_hz


def range_axis_from_frequency(frequency_hz, sweep_points):
    """
    One-way range of every rfft bin:

        R_k = k * c / (2 * N * |delta_f|)
    """

    n_bins = sweep_points // 2 + 1

    return (
        np.arange(n_bins)
        * C
        / (2.0 * sweep_points * frequency_step(frequency_hz))
    )


def design_highpass(
    cutoff_m,
    order,
    frequency_step_hz,
    sweep_points,
    sample_interval_s,
):
    """
    Butterworth high-pass that suppresses everything closer than
    cutoff_m (the antenna coupling). Raises ValueError if the filter
    cannot be used with these settings.
    """

    max_range_m = C / (4.0 * frequency_step_hz)
    cutoff_normalized = cutoff_m / max_range_m

    if not 0.0 < cutoff_normalized < 1.0:
        raise ValueError(
            f"The high-pass cutoff ({cutoff_m:g} m) must lie between "
            f"0 and the maximum range ({max_range_m:.3f} m)."
        )

    b, a = butter(order, cutoff_normalized, btype="highpass")

    pole_radius = float(np.max(np.abs(np.roots(a))))

    if not pole_radius < 1.0:
        raise ValueError(
            f"A Butterworth order of {order} is numerically unstable "
            f"for a {cutoff_m:g} m cutoff; use a lower order."
        )

    decay_samples = math.ceil(
        math.log(HIGHPASS_PADDING_DECAY) / math.log(pole_radius)
    )

    # filtfilt requires padlen < sweep_points - 1.
    padlen = max(0, min(decay_samples, sweep_points - 2))

    # Noise plus a strong stopband tone, like the coupling.
    n = np.arange(sweep_points)
    test = (
        np.random.default_rng(0).standard_normal(sweep_points)
        + 100.0 * np.cos(0.5 * math.pi * cutoff_normalized * n)
    )

    sos = butter(order, cutoff_normalized, btype="highpass", output="sos")

    with np.errstate(all="ignore"):

        reference = sosfiltfilt(sos, test, padtype="odd", padlen=padlen)
        result = filtfilt(b, a, test, padtype="odd", padlen=padlen)

        relative_error = float(
            np.max(np.abs(result - reference))
            / np.max(np.abs(reference))
        )

    if not relative_error <= HIGHPASS_MAX_RELATIVE_ERROR:
        raise ValueError(
            f"A Butterworth order of {order} is numerically inaccurate "
            f"with filtfilt for a {cutoff_m:g} m cutoff and "
            f"{sweep_points} sweep points (relative error "
            f"{relative_error:.1e}); use a lower order."
        )

    return HighpassFilter(
        cutoff_m=float(cutoff_m),
        order=int(order),
        max_range_m=max_range_m,
        cutoff_normalized=cutoff_normalized,
        cutoff_if_hz=cutoff_normalized / (2.0 * float(sample_interval_s)),
        b=b,
        a=a,
        padlen=padlen,
    )


def highpass_frames(highpass, raw, background_profile=None):
    """
    filtfilt over (n_frames, sweep_count, sweep_points), in chunks
    to bound the float64 working memory. Returns float32.
    """

    filtered = np.empty(raw.shape, dtype=np.float32)

    for start in range(0, raw.shape[0], HIGHPASS_FRAMES_PER_CHUNK):

        stop = start + HIGHPASS_FRAMES_PER_CHUNK
        block = raw[start:stop].astype(np.float64)

        if background_profile is not None:
            block -= background_profile

        filtered[start:stop] = highpass.apply(block)

    return filtered


def beamformed_frames(beamformer, measurement, background=None, highpass=None):
    """
    Beamformed complex IF of every recorded frame, with the same
    preprocessing as the display. Shape (n_frames, sweep_count,
    sweep_points), complex64.
    """

    traces = list(beamformer.trace_map)

    background_if = (
        background_matrix(background, traces)
        if background is not None
        else None
    )

    processor = RangeProcessor(
        measurement.sweep_points, background_if, highpass,
    )

    combined = np.empty(
        (
            measurement.n_frames,
            measurement.sweep_count,
            measurement.sweep_points,
        ),
        dtype=np.complex64,
    )

    frame_bytes = (
        measurement.sweep_count * len(traces) * measurement.sweep_points * 8
    )
    frames_per_chunk = max(1, PROCESSING_CHUNK_BYTES // frame_bytes)

    for start in range(0, measurement.n_frames, frames_per_chunk):

        stop = start + frames_per_chunk

        raw = np.stack(
            [getattr(measurement, f"rx{trace}")[start:stop] for trace in traces],
            axis=2,
        )

        combined[start:stop] = beamformer.combine(processor.preprocess(raw))

    return combined


def background_matrix(background, trace_map):
    """
    Averaged background IF in trace_map order.

    Shape:
        (n_traces, sweep_points)
    """

    rows = []

    for trace in trace_map:

        profile = getattr(
            background,
            f"rx{trace}_background",
            None,
        )

        if profile is None:
            raise RuntimeError(
                f"The background contains no data "
                f"for trace {trace}."
            )

        rows.append(
            np.asarray(profile, dtype=np.float64)
        )

    return np.stack(rows, axis=0)


class RangeProcessor:
    """
    Processing chain of real_time_detection_dud.py:

        raw IF
            -> background subtraction (optional)
            -> per-sweep DC removal
            -> high-pass filter with filtfilt (optional)
            -> Kaiser window
            -> FFT
            -> coherent average over the sweeps of a frame (if there are multiple sweeps)

    With a beamformer, the preprocessed traces are also combined into
    one beamformed range profile.
    """

    def __init__(
        self,
        sweep_points,
        background_if=None,
        highpass=None,
        beamformer=None,
    ):

        self.window = np.kaiser(sweep_points, KAISER_BETA)
        self.window_sum = float(np.sum(self.window))
        self.n_bins = sweep_points // 2 + 1
        self.background_if = background_if
        self.highpass = highpass
        self.beamformer = beamformer

    def preprocess(self, samples):
        """
        Background subtraction, DC removal and high-pass filter of
        samples (..., n_traces, sweep_points). Returns float64 IF.
        """

        if_data = samples.astype(np.float64)

        if self.background_if is not None:
            if_data -= self.background_if

        if_data -= np.mean(if_data, axis=-1, keepdims=True)

        if self.highpass is not None:
            if_data = self.highpass.apply(if_data)

        return if_data

    def range_profiles(self, if_data):
        """
        if_data: (sweep_count, n_traces, sweep_points)

        Returns complex range profiles, shape (n_traces, n_bins).
        """

        spectra = (
            np.fft.rfft(if_data * self.window, axis=-1)
            / self.window_sum
        )

        return np.mean(spectra, axis=0)

    def beamformed_profile(self, if_data):
        """
        if_data: (sweep_count, n_traces, sweep_points)

        Returns the complex beamformed range profile, shape (n_bins,).
        """

        combined = self.beamformer.combine(if_data)

        # The combined IF is complex: full FFT, positive ranges only.
        spectra = (
            np.fft.fft(combined * self.window, axis=-1)[..., :self.n_bins]
            / self.window_sum
        )

        return np.mean(spectra, axis=0)

    def process(self, samples):
        """
        samples: (sweep_count, n_traces, sweep_points)

        Returns complex range profiles, shape (n_traces, n_bins).
        """

        return self.range_profiles(self.preprocess(samples))


def calibrate_beamformer(
    frame_if,
    trace_map,
    window,
    range_axis_m,
    search_min_m,
    search_max_m,
    full_scale,
):
    """
    Calibrate the beamformer on a static target.

    frame_if: (n_frames, n_traces, sweep_points) preprocessed IF of a
    static scene, averaged over the sweeps of each frame.

    The target is the strongest response, summed over the traces,
    between search_min_m and search_max_m. Each trace's delay, phase
    and amplitude are measured at its own sub-bin target peak; its noise
    comes from frame differences, which cancel the static scene. Trace 0
    (or else the first trace) is the range and phase reference.

    Raises ValueError if no calibration is possible.
    """

    n_frames, n_traces, sweep_points = frame_if.shape
    trace_map = tuple(int(trace) for trace in trace_map)

    if n_traces < 2:
        raise ValueError("Beamforming needs at least two traces.")

    if len(trace_map) != n_traces:
        raise ValueError("The trace map does not match the data.")

    if n_frames < 2:
        raise ValueError("The calibration needs at least two frames.")

    reference = trace_map.index(0) if 0 in trace_map else 0
    weights_window = window / np.sum(window)
    n = np.arange(sweep_points)
    range_bin_m = float(range_axis_m[1] - range_axis_m[0])

    static = np.mean(frame_if, axis=0)
    spectrum = np.fft.rfft(static * weights_window, axis=-1)

    search = np.flatnonzero(
        (range_axis_m >= search_min_m) & (range_axis_m <= search_max_m)
    )

    if search.size == 0:
        raise ValueError(
            f"No range bins lie between {search_min_m:g} and "
            f"{search_max_m:g} m."
        )

    coarse = int(
        search[np.argmax(np.sum(np.abs(spectrum[:, search]) ** 2, axis=0))]
    )

    # Sub-bin peak of each trace near the common peak; the traces see
    # the target at slightly different ranges (internal delays).
    pad = BEAMFORMING_ZERO_PADDING
    padded = np.abs(
        np.fft.rfft(static * weights_window, n=pad * sweep_points, axis=-1)
    )
    first = max(1, (coarse - BEAMFORMING_PEAK_SEARCH_BINS) * pad)
    last = min(
        padded.shape[-1] - 2,
        (coarse + BEAMFORMING_PEAK_SEARCH_BINS) * pad,
    )

    positions = np.empty(n_traces)

    for k in range(n_traces):

        i = first + int(np.argmax(padded[k, first:last + 1]))
        a, b, c = np.log(np.maximum(padded[k, i - 1:i + 2], 1e-300))
        curvature = a - 2.0 * b + c
        offset = 0.5 * (a - c) / curvature if curvature < 0.0 else 0.0
        positions[k] = (i + float(np.clip(offset, -0.5, 0.5))) / pad

    def dtft(x, position):
        """Windowed spectrum of x at a fractional bin position."""

        return np.sum(
            x * weights_window
            * np.exp(-2j * np.pi * position * n / sweep_points),
            axis=-1,
        )

    h = np.array([dtft(static[k], positions[k]) for k in range(n_traces)])

    h_frames = np.stack(
        [dtft(frame_if[:, k, :], positions[k]) for k in range(n_traces)],
        axis=1,
    )

    shift_bins = positions - positions[reference]
    target_range_m = float(positions[reference] * range_bin_m)

    pairs = n_frames // 2
    differences = (
        frame_if[1:2 * pairs:2] - frame_if[0:2 * pairs:2]
    ) / math.sqrt(2.0)

    noise_bins = (range_axis_m >= search_min_m) & (
        np.abs(range_axis_m - target_range_m)
        > BEAMFORMING_TARGET_EXCLUSION_M
    )

    if not np.any(noise_bins):
        raise ValueError("No range bins are left for the noise estimate.")

    noise_spectra = np.fft.rfft(differences * weights_window, axis=-1)
    noise_power = np.mean(
        np.abs(noise_spectra[:, :, noise_bins]) ** 2, axis=(0, 2),
    )

    if not np.all(noise_power > 0.0):
        raise ValueError("The noise power could not be estimated.")

    amplitude = np.abs(h) / noise_power
    amplitude *= n_traces / np.sum(amplitude)

    weights = amplitude * np.exp(1j * (np.angle(h[reference]) - np.angle(h)))

    steering = weights[:, None] * np.exp(
        -2j * np.pi * shift_bins[:, None] * n[None, :] / sweep_points
    )

    # Measured on the calibration frames, as the display computes it.
    combined_noise_spectra = np.fft.fft(
        np.einsum("pkn,kn->pn", differences, steering) * weights_window,
        axis=-1,
    )[:, :range_axis_m.size]

    combined_noise_power = np.mean(
        np.abs(combined_noise_spectra[:, noise_bins]) ** 2
    )

    combined_snr_db = 10.0 * math.log10(
        abs(np.sum(weights * h)) ** 2 / combined_noise_power
    )

    relative = h_frames * np.conj(h_frames[:, reference:reference + 1])
    mean_phase = np.angle(np.mean(relative, axis=0))
    phase_std_deg = np.degrees(
        np.sqrt(
            np.mean(np.angle(relative * np.exp(-1j * mean_phase)) ** 2, axis=0)
        )
    )

    scale = 2.0 / float(full_scale)

    return Beamformer(
        trace_map=trace_map,
        reference_trace=trace_map[reference],
        shift_bins=shift_bins,
        weights=weights,
        steering=steering,
        range_bin_m=range_bin_m,
        target_range_m=target_range_m,
        target_dbfs=20.0 * np.log10(np.abs(h) * scale),
        noise_dbfs=10.0 * np.log10(noise_power * scale**2),
        channel_snr_db=10.0 * np.log10(np.abs(h) ** 2 / noise_power),
        phase_std_deg=phase_std_deg,
        combined_snr_db=combined_snr_db,
        n_frames=n_frames,
    )


# =====================================================================
# Recording and saving
# =====================================================================

class FrameRecorder:
    """Stores a fixed number of frames in preallocated arrays."""

    def __init__(self, n_frames, first_frame):

        sweep_count, _, sweep_points = first_frame.samples.shape

        self.n_frames = n_frames
        self.count = 0

        self.trace_map = list(first_frame.trace_map)
        self.sweep_count = sweep_count
        self.sweep_points = sweep_points
        self.timestamp_utc = first_frame.timestamp_utc

        self.rx = {
            trace: np.empty(
                (n_frames, sweep_count, sweep_points),
                dtype=first_frame.samples.dtype,
            )
            for trace in self.trace_map
        }

        self.frame_timestamps_s = np.full(n_frames, np.nan)

        self.trigger_timestamps_ns = np.full(
            n_frames,
            -1,
            dtype=np.int64,
        )

    @property
    def full(self):
        return self.count >= self.n_frames

    def add(self, frame):

        index = self.count

        if index == 0:
            self.timestamp_utc = frame.timestamp_utc

        for trace_index, trace in enumerate(frame.trace_map):
            self.rx[trace][index] = frame.samples[:, trace_index, :]

        self.frame_timestamps_s[index] = datetime.fromisoformat(
            frame.timestamp_utc
        ).timestamp()

        if frame.trigger_timestamp_ns is not None:
            self.trigger_timestamps_ns[index] = (
                frame.trigger_timestamp_ns
            )

        self.count += 1

    def to_measurement(self):

        n_frames = self.count

        rx = {
            trace: self.rx[trace][:n_frames]
            for trace in self.trace_map
        }

        return RadarMeasurement(
            timestamp_utc=self.timestamp_utc,
            n_frames=n_frames,
            sweep_count=self.sweep_count,
            sweep_points=self.sweep_points,
            traces=list(self.trace_map),
            rx0=rx.get(0),
            rx1=rx.get(1),
            rx2=rx.get(2),
        )


def build_save_content(
    measurement: RadarMeasurement,
    config: RadarConfig,
    background: RadarBackground | None,
    frequency_hz,
    range_axis_m,
    frame_timestamps_s,
    trigger_timestamps_ns,
    full_scale,
    n_frames_requested,
    metadata,
    highpass: HighpassFilter | None = None,
    beamformer: Beamformer | None = None,
):
    """Collect everything that is written to the output file."""

    content = {
        "timestamp_utc": measurement.timestamp_utc,
        "n_frames": int(measurement.n_frames),
        "n_frames_requested": int(n_frames_requested),
        "recording_complete": bool(
            measurement.n_frames >= n_frames_requested
        ),
        "sweep_count": int(measurement.sweep_count),
        "sweep_points": int(measurement.sweep_points),
        "traces": np.asarray(measurement.traces, dtype=np.int64),
        "frequency_hz": np.asarray(frequency_hz, dtype=np.float64),
        "range_axis_m": np.asarray(range_axis_m, dtype=np.float64),
        "frame_timestamps_s": np.asarray(
            frame_timestamps_s,
            dtype=np.float64,
        ),
        "trigger_timestamps_ns": np.asarray(
            trigger_timestamps_ns,
            dtype=np.int64,
        ),
        "full_scale": (
            float(full_scale)
            if full_scale is not None
            else float("nan")
        ),
        "background_removed": background is not None,
        "highpass_filtered": highpass is not None,
        "beamformed": beamformer is not None,
    }

    for trace in measurement.traces:
        content[f"rx{trace}"] = getattr(measurement, f"rx{trace}")

    if background is not None:

        content["background_timestamp_utc"] = background.timestamp_utc
        content["background_n_acquisitions"] = int(
            background.n_acquisitions
        )

        for trace in measurement.traces:

            profile = np.asarray(
                getattr(background, f"rx{trace}_background"),
                dtype=np.float64,
            )

            raw = getattr(measurement, f"rx{trace}")

            content[f"rx{trace}_background"] = profile

            background_raw = getattr(background, f"rx{trace}_raw")

            if background_raw is not None:
                content[f"rx{trace}_background_raw"] = background_raw

            content[f"rx{trace}_background_removed"] = (
                raw.astype(np.float32)
                - profile.astype(np.float32)
            )

    if highpass is not None:

        content["highpass_cutoff_m"] = highpass.cutoff_m
        content["highpass_cutoff_if_hz"] = highpass.cutoff_if_hz
        content["highpass_cutoff_normalized"] = highpass.cutoff_normalized
        content["highpass_order"] = highpass.order
        content["highpass_b"] = highpass.b
        content["highpass_a"] = highpass.a
        content["highpass_padlen"] = highpass.padlen
        content["highpass_method"] = (
            "scipy.signal.filtfilt(highpass_b, highpass_a, x, axis=-1, "
            "padtype='odd', padlen=highpass_padlen)"
        )

        for trace in measurement.traces:

            profile = None

            if background is not None:
                profile = np.asarray(
                    getattr(background, f"rx{trace}_background"),
                    dtype=np.float64,
                )

            content[f"rx{trace}_highpass"] = highpass_frames(
                highpass,
                getattr(measurement, f"rx{trace}"),
                profile,
            )

    if beamformer is not None:

        content["beamforming_traces"] = np.asarray(
            beamformer.trace_map, dtype=np.int64,
        )
        content["beamforming_reference_trace"] = beamformer.reference_trace
        content["beamforming_weights"] = beamformer.weights
        content["beamforming_shift_bins"] = beamformer.shift_bins
        content["beamforming_target_range_m"] = beamformer.target_range_m
        content["beamforming_channel_snr_db"] = beamformer.channel_snr_db
        content["beamforming_phase_std_deg"] = beamformer.phase_std_deg
        content["beamforming_snr_gain_db"] = beamformer.snr_gain_db
        content["beamforming_calibration_frames"] = beamformer.n_frames
        content["beamforming_method"] = (
            "rx_beamformed[..., n] = sum_k beamforming_weights[k] * x_k[..., n]"
            " * exp(-2j*pi*beamforming_shift_bins[k]*n/sweep_points), k over"
            " beamforming_traces; x_k = rx_k minus rx_k_background (if"
            " removed), minus its mean over each sweep, high-pass filtered"
            " (if on)"
        )
        content["rx_beamformed"] = beamformed_frames(
            beamformer, measurement, background, highpass,
        )

    for config_field in fields(config):

        value = getattr(config, config_field.name)

        if value is None:
            continue

        if isinstance(value, (list, tuple)):
            value = np.asarray(value)

        content[f"config_{config_field.name}"] = value

    for key, value in metadata.items():
        content[key] = json.dumps(value, default=str)

    return content


def write_acquisition_file(path, content):
    """Write content as .npz (NumPy) or .mat (MATLAB)."""

    if path.suffix == ".npz":

        np.savez_compressed(path, **content)

    elif path.suffix == ".mat":

        from scipy.io import savemat

        savemat(
            str(path),
            content,
            do_compression=True,
            oned_as="row",
        )

    else:
        raise ValueError(
            f"Unsupported file type: {path.suffix}"
        )


def unique_output_path(folder, suffix):

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    candidate = folder / f"acquisition_{stamp}{suffix}"
    index = 1

    while candidate.exists():
        candidate = folder / f"acquisition_{stamp}_{index}{suffix}"
        index += 1

    return candidate


def check_output_folder(folder):
    """Create the folder if needed and verify that it is writable."""

    folder.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryFile(dir=folder):
        pass


def read_setup_report():

    try:
        return json.loads(
            HEALTH_REPORT_FILE.read_text(encoding="utf-8")
        )

    except (OSError, ValueError) as error:
        return {
            "error": f"Could not read {HEALTH_REPORT_FILE}: {error}"
        }


def query_device_info(device):

    try:
        return device.get_info()

    except Exception as error:
        return {"error": f"{type(error).__name__}: {error}"}


# =====================================================================
# GUI
# =====================================================================

class RadarGui:

    def __init__(self, root):

        self.root = root
        self.root.title("X1000 Radar - Acquisition")

        width = min(1400, self.root.winfo_screenwidth() - 80)
        height = min(900, self.root.winfo_screenheight() - 120)
        self.root.geometry(f"{width}x{height}+20+20")
        self.root.minsize(1000, 640)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._events = queue.Queue()
        self._stop_event = threading.Event()

        self._display_lock = threading.Lock()
        self._pending_display = None

        self._worker = None
        self._running = False
        self._close_requested = False
        self._alive = True

        self._run_id = 0
        self._current_settings = None
        self._last_saved_path = None

        self._plot = None

        self._create_variables()
        self._build_layout()
        self._update_widget_states()
        self._show_message_plot("Set the parameters and press Start.")

        # The window must stay tall enough to show the Start/Stop buttons.
        self.root.update_idletasks()
        self.root.minsize(
            1000,
            max(
                640,
                min(
                    self._controls.winfo_reqheight(),
                    self.root.winfo_screenheight() - 80,
                ),
            ),
        )

        self._log(
            "Ready. Set the radar parameters, answer the four "
            "questions on the left and press Start."
        )

        self.root.after(POLL_INTERVAL_MS, self._poll_events)

    # -----------------------------------------------------------------
    # Layout
    # -----------------------------------------------------------------

    def _create_variables(self):

        self.values = {
            key: tk.StringVar(value=value)
            for key, value in DEFAULT_VALUES.items()
        }

        self.trace_enabled = {
            trace: tk.BooleanVar(value=True)
            for trace in (0, 1, 2)
        }

        self.remove_background = tk.StringVar(value="yes")
        self.highpass = tk.StringVar(value="no")
        self.beamforming = tk.StringVar(value="no")
        self.output_mode = tk.StringVar(value="display")
        self.output_dir = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.file_format = tk.StringVar(value="npz")

        self.status_text = tk.StringVar(value="Idle.")
        self.display_status_text = tk.StringVar(value="")

    def _build_layout(self):

        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.root, padding=8)
        controls.grid(row=0, column=0, sticky="ns")
        self._controls = controls

        right = ttk.Frame(self.root, padding=(0, 8, 8, 8))
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=4)
        right.rowconfigure(3, weight=1)

        self._input_widgets = []
        self._background_widgets = []
        self._highpass_widgets = []
        self._beamforming_widgets = []
        self._save_widgets = []
        self._display_widgets = []

        self._build_parameter_frame(controls)
        self._build_background_frame(controls)
        self._build_highpass_frame(controls)
        self._build_beamforming_frame(controls)
        self._build_output_frame(controls)
        self._build_display_frame(controls)
        self._build_run_frame(controls)

        self._build_plot_area(right)

    def _add_entry_row(self, parent, row, label, variable, unit):

        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", padx=(0, 6), pady=2,
        )

        entry = ttk.Entry(
            parent, textvariable=variable, width=10, justify="right",
        )
        entry.grid(row=row, column=1, sticky="ew", pady=2)

        ttk.Label(parent, text=unit).grid(
            row=row, column=2, sticky="w", padx=(4, 0), pady=2,
        )

        self._input_widgets.append(entry)

        return entry

    def _build_parameter_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="1. Radar parameters", padding=8,
        )
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        rows = (
            ("f_start_ghz", "Start frequency", "GHz"),
            ("f_stop_ghz", "Stop frequency", "GHz"),
            ("sweep_time_ms", "Sweep time", "ms"),
            ("sweep_period_ms", "Sweep period", "ms"),
            ("sweep_count", "Sweeps per frame", ""),
        )

        for row, (key, label, unit) in enumerate(rows):
            self._add_entry_row(frame, row, label, self.values[key], unit)

        ttk.Label(frame, text="Traces").grid(
            row=len(rows), column=0, sticky="w", pady=(4, 0),
        )

        traces = ttk.Frame(frame)
        traces.grid(
            row=len(rows), column=1, columnspan=2, sticky="w", pady=(4, 0),
        )

        for column, trace in enumerate((0, 1, 2)):

            check = ttk.Checkbutton(
                traces,
                text=TRACE_LABELS[trace],
                variable=self.trace_enabled[trace],
            )
            check.grid(row=0, column=column, sticky="w", padx=(0, 6))

            self._input_widgets.append(check)

    def _build_background_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="2. Background removal", padding=8,
        )
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame, text="Do you want the background to be removed?",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        choices = ttk.Frame(frame)
        choices.grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 4))

        for column, (value, text) in enumerate(
            (("yes", "Yes"), ("no", "No"))
        ):

            radio = ttk.Radiobutton(
                choices,
                text=text,
                value=value,
                variable=self.remove_background,
                command=self._update_widget_states,
            )
            radio.grid(row=0, column=column, sticky="w", padx=(0, 16))

            self._input_widgets.append(radio)

        entry = self._add_entry_row(
            frame,
            2,
            "Background acquisitions",
            self.values["background_acquisitions"],
            "",
        )

        self._background_widgets.append(entry)

    def _build_highpass_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="3. Coupling high-pass filter", padding=8,
        )
        frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame, text="Do you want the coupling to be high-pass filtered?",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        choices = ttk.Frame(frame)
        choices.grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 4))

        for column, (value, text) in enumerate(
            (("yes", "Yes"), ("no", "No"))
        ):

            radio = ttk.Radiobutton(
                choices,
                text=text,
                value=value,
                variable=self.highpass,
                command=self._update_widget_states,
            )
            radio.grid(row=0, column=column, sticky="w", padx=(0, 16))

            self._input_widgets.append(radio)

        rows = (
            ("highpass_cutoff_m", "Cutoff range", "m"),
            ("highpass_order", "Butterworth order", ""),
        )

        for row, (key, label, unit) in enumerate(rows, start=2):

            entry = self._add_entry_row(
                frame, row, label, self.values[key], unit,
            )

            self._highpass_widgets.append(entry)

    def _build_beamforming_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="4. RX beamforming", padding=8,
        )
        frame.grid(row=3, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame, text="Do you want the RX traces to be combined coherently?",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        choices = ttk.Frame(frame)
        choices.grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 4))

        for column, (value, text) in enumerate(
            (("yes", "Yes"), ("no", "No"))
        ):

            radio = ttk.Radiobutton(
                choices,
                text=text,
                value=value,
                variable=self.beamforming,
                command=self._update_widget_states,
            )
            radio.grid(row=0, column=column, sticky="w", padx=(0, 16))

            self._input_widgets.append(radio)

        entry = self._add_entry_row(
            frame,
            2,
            "Calibration frames",
            self.values["bf_calibration_frames"],
            "",
        )

        self._beamforming_widgets.append(entry)

        ttk.Label(frame, text="Calibration target").grid(
            row=3, column=0, sticky="w", padx=(0, 6), pady=2,
        )

        window = ttk.Frame(frame)
        window.grid(row=3, column=1, columnspan=2, sticky="w", pady=2)

        for column, key in ((0, "bf_search_min_m"), (2, "bf_search_max_m")):

            entry = ttk.Entry(
                window,
                textvariable=self.values[key],
                width=6,
                justify="right",
            )
            entry.grid(row=0, column=column)

            self._input_widgets.append(entry)
            self._beamforming_widgets.append(entry)

        ttk.Label(window, text="to").grid(row=0, column=1, padx=4)
        ttk.Label(window, text="m").grid(row=0, column=3, padx=(4, 0))

    def _build_output_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="5. Acquisition output", padding=8,
        )
        frame.grid(row=4, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame, text="What do you want to do with the acquisition?",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        choices = ttk.Frame(frame)
        choices.grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 4))

        for row, (value, text) in enumerate(
            (
                ("save", "Save it"),
                ("display", "Display it in real time"),
                ("both", "Both"),
            )
        ):

            radio = ttk.Radiobutton(
                choices,
                text=text,
                value=value,
                variable=self.output_mode,
                command=self._update_widget_states,
            )
            radio.grid(row=row, column=0, sticky="w")

            self._input_widgets.append(radio)

        entry = self._add_entry_row(
            frame,
            2,
            "Frames to record",
            self.values["frames_to_record"],
            "",
        )

        self._save_widgets.append(entry)

        ttk.Label(frame, text="Save folder").grid(
            row=3, column=0, sticky="w", pady=(4, 0),
        )

        folder_entry = ttk.Entry(
            frame, textvariable=self.output_dir, width=30,
        )
        folder_entry.grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)

        browse = ttk.Button(
            frame, text="Browse...", command=self._browse_output_dir,
        )
        browse.grid(row=4, column=2, sticky="w", padx=(4, 0), pady=2)

        ttk.Label(frame, text="File format").grid(
            row=5, column=0, sticky="w", pady=2,
        )

        formats = ttk.Frame(frame)
        formats.grid(row=5, column=1, columnspan=2, sticky="w", pady=2)

        format_widgets = []

        for column, (value, text) in enumerate(
            (("npz", "NumPy (.npz)"), ("mat", "MATLAB (.mat)"))
        ):

            radio = ttk.Radiobutton(
                formats,
                text=text,
                value=value,
                variable=self.file_format,
            )
            radio.grid(row=0, column=column, sticky="w", padx=(0, 8))

            format_widgets.append(radio)

        for widget in (folder_entry, browse, *format_widgets):
            self._input_widgets.append(widget)
            self._save_widgets.append(widget)

    def _build_display_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="Real-time display", padding=8,
        )
        frame.grid(row=5, column=0, sticky="ew", pady=(0, 8))

        ttk.Label(frame, text="from").grid(row=0, column=1)
        ttk.Label(frame, text="to").grid(row=0, column=2)

        rows = (
            ("Plot range [m]", "plot_min_range_m", "plot_max_range_m"),
            ("Plot level [dBFS]", "plot_min_db", "plot_max_db"),
            ("Peak search [m]", "peak_min_range_m", "peak_max_range_m"),
        )

        for row, (label, low_key, high_key) in enumerate(rows, start=1):

            ttk.Label(frame, text=label).grid(
                row=row, column=0, sticky="w", padx=(0, 6), pady=2,
            )

            for column, key in ((1, low_key), (2, high_key)):

                entry = ttk.Entry(
                    frame,
                    textvariable=self.values[key],
                    width=8,
                    justify="right",
                )
                entry.grid(row=row, column=column, padx=2, pady=2)

                self._input_widgets.append(entry)
                self._display_widgets.append(entry)

    def _build_run_frame(self, parent):

        frame = ttk.Frame(parent)
        frame.grid(row=6, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        self.start_button = ttk.Button(
            frame, text="Start", command=self._on_start,
        )
        self.start_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.stop_button = ttk.Button(
            frame, text="Stop", command=self._on_stop,
        )
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.progress = ttk.Progressbar(
            frame, mode="determinate", maximum=1.0,
        )
        self.progress.grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(8, 2),
        )

        ttk.Label(
            frame,
            textvariable=self.status_text,
            wraplength=300,
            justify="left",
        ).grid(row=2, column=0, columnspan=2, sticky="w")

    def _build_plot_area(self, parent):

        plot_frame = ttk.Frame(parent)
        plot_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)

        self.figure = Figure(figsize=(9, 6), layout="constrained")

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # The live curves are blitted; see _on_canvas_draw.
        self.canvas.mpl_connect("draw_event", self._on_canvas_draw)
        self.canvas.mpl_connect("resize_event", self._on_canvas_resize)

        self.toolbar = NavigationToolbar2Tk(
            self.canvas, plot_frame, pack_toolbar=False,
        )
        self.toolbar.update()
        self.toolbar.grid(row=1, column=0, sticky="ew")

        ttk.Label(
            parent, textvariable=self.display_status_text, anchor="w",
        ).grid(row=1, column=0, sticky="ew", pady=4)

        ttk.Label(parent, text="Log").grid(row=2, column=0, sticky="w")

        self.log_text = ScrolledText(
            parent, height=10, state="disabled", wrap="word",
        )
        self.log_text.grid(row=3, column=0, sticky="nsew")

    # -----------------------------------------------------------------
    # Widget helpers
    # -----------------------------------------------------------------

    @staticmethod
    def _set_enabled(widget, enabled):
        widget.state(["!disabled"] if enabled else ["disabled"])

    def _update_widget_states(self):

        running = self._running
        mode = self.output_mode.get()

        save = mode in ("save", "both")
        display = mode in ("display", "both")
        background = self.remove_background.get() == "yes"
        highpass = self.highpass.get() == "yes"
        beamforming = self.beamforming.get() == "yes"

        for widget in self._input_widgets:
            self._set_enabled(widget, not running)

        for widget in self._background_widgets:
            self._set_enabled(widget, not running and background)

        for widget in self._highpass_widgets:
            self._set_enabled(widget, not running and highpass)

        for widget in self._beamforming_widgets:
            self._set_enabled(widget, not running and beamforming)

        for widget in self._save_widgets:
            self._set_enabled(widget, not running and save)

        for widget in self._display_widgets:
            self._set_enabled(widget, not running and display)

        self._set_enabled(self.start_button, not running)

        self._set_enabled(
            self.stop_button,
            running and not self._stop_event.is_set(),
        )

    def _browse_output_dir(self):

        folder = filedialog.askdirectory(
            parent=self.root,
            initialdir=self.output_dir.get() or str(SCRIPT_DIR),
            title="Choose the save folder",
        )

        if folder:
            self.output_dir.set(folder)

    def _set_status(self, text):
        self.status_text.set(text)

    def _log(self, message):

        stamp = datetime.now().strftime("%H:%M:%S")

        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n")

        line_count = int(self.log_text.index("end-1c").split(".")[0])

        if line_count > MAX_LOG_LINES:
            self.log_text.delete(
                "1.0", f"{line_count - MAX_LOG_LINES + 1}.0",
            )

        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # -----------------------------------------------------------------
    # Parameter input
    # -----------------------------------------------------------------

    def _number(self, key, label, positive=False, minimum=None):

        raw_text = self.values[key].get()
        text = raw_text.strip().replace(",", ".")

        try:
            value = float(text)
        except ValueError:
            raise ValueError(
                f"{label}: '{raw_text}' is not a number."
            ) from None

        if not math.isfinite(value):
            raise ValueError(f"{label} must be a finite number.")

        if positive and value <= 0.0:
            raise ValueError(f"{label} must be greater than zero.")

        if minimum is not None and value < minimum:
            raise ValueError(f"{label} must be at least {minimum:g}.")

        return value

    def _integer(self, key, label, minimum, maximum=None):

        text = self.values[key].get().strip()

        try:
            value = int(text)
        except ValueError:
            raise ValueError(
                f"{label}: '{text}' is not a whole number."
            ) from None

        if value < minimum:
            raise ValueError(f"{label} must be at least {minimum}.")

        if maximum is not None and value > maximum:
            raise ValueError(f"{label} must be at most {maximum}.")

        return value

    def _read_settings(self):
        """Validate the inputs. Raises ValueError with a readable message."""

        mode = self.output_mode.get()
        save = mode in ("save", "both")
        display = mode in ("display", "both")

        remove_background = self.remove_background.get() == "yes"

        f_start_ghz = self._number(
            "f_start_ghz", "Start frequency", positive=True,
        )
        f_stop_ghz = self._number(
            "f_stop_ghz", "Stop frequency", positive=True,
        )

        if f_stop_ghz <= f_start_ghz:
            raise ValueError(
                "The stop frequency must be greater than "
                "the start frequency."
            )

        sweep_time_ms = self._number(
            "sweep_time_ms", "Sweep time", positive=True,
        )
        sweep_period_ms = self._number(
            "sweep_period_ms", "Sweep period", positive=True,
        )

        if sweep_period_ms < sweep_time_ms:
            raise ValueError(
                "The sweep period must be at least as long "
                "as the sweep time."
            )

        sweep_count = self._integer(
            "sweep_count", "Sweeps per frame", minimum=1,
        )

        traces = tuple(
            trace
            for trace in (0, 1, 2)
            if self.trace_enabled[trace].get()
        )

        if not traces:
            raise ValueError("Select at least one trace.")

        background_acquisitions = 0

        if remove_background:
            background_acquisitions = self._integer(
                "background_acquisitions",
                "Background acquisitions",
                minimum=1,
            )

        highpass = self.highpass.get() == "yes"
        highpass_cutoff_m = 0.0
        highpass_order = 0

        if highpass:

            # The cutoff is checked against the maximum range once the
            # radar reports its frequency step.
            highpass_cutoff_m = self._number(
                "highpass_cutoff_m", "High-pass cutoff range", positive=True,
            )
            highpass_order = self._integer(
                "highpass_order",
                "Butterworth order",
                minimum=1,
                maximum=HIGHPASS_MAX_ORDER,
            )

        beamforming = self.beamforming.get() == "yes"
        bf_calibration_frames = 0
        bf_search_min_m = 0.0
        bf_search_max_m = 0.0

        if beamforming:

            if len(traces) < 2:
                raise ValueError(
                    "RX beamforming needs at least two traces."
                )

            bf_calibration_frames = self._integer(
                "bf_calibration_frames",
                "Calibration frames",
                minimum=BEAMFORMING_MIN_CALIBRATION_FRAMES,
            )
            bf_search_min_m = self._number(
                "bf_search_min_m", "Calibration target (from)", minimum=0.0,
            )
            bf_search_max_m = self._number(
                "bf_search_max_m", "Calibration target (to)",
            )

            if bf_search_max_m <= bf_search_min_m:
                raise ValueError(
                    "Calibration target: 'to' must be greater than 'from'."
                )

        frames_to_record = 0
        output_dir = DEFAULT_OUTPUT_DIR

        if save:

            frames_to_record = self._integer(
                "frames_to_record", "Frames to record", minimum=1,
            )

            folder_text = self.output_dir.get().strip()

            if not folder_text:
                raise ValueError("Choose a save folder.")

            output_dir = Path(folder_text).expanduser().resolve()

            try:
                check_output_folder(output_dir)
            except OSError as error:
                raise ValueError(
                    f"The save folder cannot be used:\n"
                    f"{output_dir}\n\n{error}"
                ) from None

        plot_limits = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        if display:

            plot_min_range_m = self._number(
                "plot_min_range_m", "Plot range (from)", minimum=0.0,
            )
            plot_max_range_m = self._number(
                "plot_max_range_m", "Plot range (to)",
            )
            plot_min_db = self._number(
                "plot_min_db", "Plot level (from)",
            )
            plot_max_db = self._number(
                "plot_max_db", "Plot level (to)",
            )
            peak_min_range_m = self._number(
                "peak_min_range_m", "Peak search (from)", minimum=0.0,
            )
            peak_max_range_m = self._number(
                "peak_max_range_m", "Peak search (to)",
            )

            if plot_max_range_m <= plot_min_range_m:
                raise ValueError(
                    "Plot range: 'to' must be greater than 'from'."
                )

            if plot_max_db <= plot_min_db:
                raise ValueError(
                    "Plot level: 'to' must be greater than 'from'."
                )

            if peak_max_range_m <= peak_min_range_m:
                raise ValueError(
                    "Peak search: 'to' must be greater than 'from'."
                )

            plot_limits = (
                plot_min_range_m,
                plot_max_range_m,
                plot_min_db,
                plot_max_db,
                peak_min_range_m,
                peak_max_range_m,
            )

        return RunSettings(
            f_start_hz=f_start_ghz * 1e9,
            f_stop_hz=f_stop_ghz * 1e9,
            sweep_time_s=sweep_time_ms / 1e3,
            sweep_period_s=sweep_period_ms / 1e3,
            sweep_count=sweep_count,
            traces=traces,
            remove_background=remove_background,
            background_acquisitions=background_acquisitions,
            highpass=highpass,
            highpass_cutoff_m=highpass_cutoff_m,
            highpass_order=highpass_order,
            beamforming=beamforming,
            bf_calibration_frames=bf_calibration_frames,
            bf_search_min_m=bf_search_min_m,
            bf_search_max_m=bf_search_max_m,
            save=save,
            display=display,
            frames_to_record=frames_to_record,
            output_dir=output_dir,
            file_format=self.file_format.get(),
            plot_min_range_m=plot_limits[0],
            plot_max_range_m=plot_limits[1],
            plot_min_db=plot_limits[2],
            plot_max_db=plot_limits[3],
            peak_min_range_m=plot_limits[4],
            peak_max_range_m=plot_limits[5],
        )

    def _log_settings(self, settings):

        if settings.save and settings.display:
            output = "save and display in real time"
        elif settings.save:
            output = "save"
        else:
            output = "display in real time"

        if settings.remove_background:
            background = (
                f"removed ({settings.background_acquisitions} "
                f"acquisitions)"
            )
        else:
            background = "not removed"

        if settings.highpass:
            highpass = (
                f"cutoff {settings.highpass_cutoff_m:g} m, Butterworth "
                f"order {settings.highpass_order}, filtfilt"
            )
        else:
            highpass = "off"

        if settings.beamforming:
            beamforming = (
                f"on (calibrated on the strongest target between "
                f"{settings.bf_search_min_m:g} and {settings.bf_search_max_m:g} "
                f"m, {settings.bf_calibration_frames} frames)"
            )
        else:
            beamforming = "off"

        lines = [
            "Starting:",
            f"  Frequency  : {settings.f_start_hz / 1e9:g} - "
            f"{settings.f_stop_hz / 1e9:g} GHz",
            f"  Sweep      : time {settings.sweep_time_s * 1e3:g} ms, "
            f"period {settings.sweep_period_s * 1e3:g} ms, "
            f"{settings.sweep_count} sweep(s) per frame",
            f"  Traces     : {list(settings.traces)}",
            f"  Background : {background}",
            f"  High-pass  : {highpass}",
            f"  Beamforming: {beamforming}",
            f"  Output     : {output}",
        ]

        if settings.save:
            lines.append(
                f"  Recording  : {settings.frames_to_record} frames as "
                f".{settings.file_format} in {settings.output_dir}"
            )

        self._log("\n".join(lines))

    # -----------------------------------------------------------------
    # Start / stop / close
    # -----------------------------------------------------------------

    def _on_start(self):

        if self._running:
            return

        try:
            settings = self._read_settings()
        except ValueError as error:
            messagebox.showerror(
                "Invalid parameter", str(error), parent=self.root,
            )
            return

        self._run_id += 1
        self._current_settings = settings
        self._last_saved_path = None

        self._stop_event.clear()

        with self._display_lock:
            self._pending_display = None

        self._running = True
        self._update_widget_states()

        self.progress.configure(maximum=1.0, value=0.0)
        self.display_status_text.set("")
        self._set_status("Starting...")
        self._show_message_plot("Connecting to the radar...")
        self._log_settings(settings)

        self._worker = threading.Thread(
            target=self._run_session,
            args=(settings, self._run_id),
            name="radar-worker",
            daemon=True,
        )
        self._worker.start()

    def _on_stop(self):

        if not self._running or self._stop_event.is_set():
            return

        self._stop_event.set()
        self._set_status("Stopping...")
        self._update_widget_states()

    def _on_close(self):

        if not self._running:
            self._destroy()
            return

        if self._close_requested:

            if messagebox.askyesno(
                "Close",
                "The radar is still stopping.\n\nClose anyway?",
                parent=self.root,
            ):
                self._destroy()

            return

        if not messagebox.askokcancel(
            "Close",
            "An acquisition is running.\n\n"
            "Stop it and close the window? Frames that were "
            "already recorded are saved first.",
            parent=self.root,
        ):
            return

        self._close_requested = True
        self._stop_event.set()
        self._set_status("Stopping before closing...")
        self._update_widget_states()

    def _destroy(self):

        self._alive = False
        self._stop_event.set()
        self.root.destroy()

    # -----------------------------------------------------------------
    # Communication with the worker thread
    # -----------------------------------------------------------------

    def _post(self, kind, *payload):
        self._events.put((kind, *payload))

    def _ask(self, title, message):
        """Ask an OK/Cancel question from the worker thread."""

        reply = queue.Queue(maxsize=1)
        self._post("ask", title, message, reply)

        while True:

            try:
                return reply.get(timeout=0.1)

            except queue.Empty:

                if not self._alive:
                    return False

    def _publish_display(self, item):

        with self._display_lock:
            self._pending_display = item

    def _make_progress_callback(self, label):

        last_post = [0.0]

        def callback(done, total):

            now = time.perf_counter()

            if done >= total or now - last_post[0] >= PROGRESS_INTERVAL_S:
                last_post[0] = now
                self._post("progress", done, total, label)

        return callback

    def _poll_events(self):

        try:

            for _ in range(MAX_EVENTS_PER_POLL):

                try:
                    event = self._events.get_nowait()
                except queue.Empty:
                    break

                self._handle_event(event)

                if not self._alive:
                    return

            with self._display_lock:
                item = self._pending_display
                self._pending_display = None

            if item is not None:
                self._draw_frame(item)

        except Exception:
            self._log("GUI error:\n" + traceback.format_exc())

        finally:
            if self._alive:
                self.root.after(POLL_INTERVAL_MS, self._poll_events)

    def _handle_event(self, event):

        kind, *payload = event

        if kind == "log":
            self._log(payload[0])

        elif kind == "status":
            self._set_status(payload[0])

        elif kind == "progress":
            done, total, label = payload
            self.progress.configure(maximum=max(total, 1), value=done)
            self._set_status(f"{label}: {done}/{total}")

        elif kind == "config":
            self._on_config(payload[0])

        elif kind == "ask":
            self._answer_question(*payload)

        elif kind == "display_init":
            self._init_plot(payload[0])

        elif kind == "saved":
            self._on_saved(*payload)

        elif kind == "error":
            self._on_error(*payload)

        elif kind == "finished":
            self._on_worker_finished(*payload)

    def _answer_question(self, title, message, reply):

        if self._stop_event.is_set() or self._close_requested:
            reply.put(False)
            return

        answer = messagebox.askokcancel(title, message, parent=self.root)
        reply.put(bool(answer))

    def _on_config(self, config):

        self._log(
            "Radar configured:\n"
            f"  Frequency    : {config.frequency_start_hz / 1e9:.6f} - "
            f"{config.frequency_stop_hz / 1e9:.6f} GHz "
            f"(bandwidth {config.bandwidth_hz / 1e9:.6f} GHz)\n"
            f"  Sweep        : time {config.sweep_time_s * 1e3:.6g} ms, "
            f"period {config.sweep_period_s * 1e3:.6g} ms, "
            f"{config.sweep_count} sweep(s) per frame\n"
            f"  Sweep points : {config.sweep_points} (sample interval "
            f"{float(config.sample_interval_s) * 1e9:.3f} ns)\n"
            f"  Range        : resolution "
            f"{config.range_resolution_m * 1e3:.3f} mm, one-sided "
            f"maximum {config.max_one_sided_range_m:.3f} m\n"
            f"  Traces       : {config.enabled_traces}\n"
            f"  PLL locked   : main {config.main_pll_locked}, "
            f"aux {config.aux_pll_locked} (state {config.radar_state})"
        )

        self._set_status("Radar configured.")

        settings = self._current_settings

        if settings is not None and not settings.display:
            self._show_message_plot(
                "Real-time display is off (save only).\n"
                "The recording progress is shown on the left."
            )
        else:
            self._show_message_plot("Waiting for the first frame...")

    def _on_saved(self, path, n_frames, n_frames_requested):

        self._last_saved_path = path

        if n_frames >= n_frames_requested:
            text = f"Saved {n_frames} frames to {path}"
        else:
            text = (
                f"Saved {n_frames} of {n_frames_requested} frames "
                f"(recording stopped early) to {path}"
            )

        self._log(text)

        settings = self._current_settings

        if (
            settings is not None
            and settings.display
            and not self._stop_event.is_set()
        ):
            self._set_status(
                text + "\n\nThe real-time display continues. "
                "Press Stop to end."
            )
        else:
            self._set_status(text)

    def _on_error(self, message, details):

        self._log(f"ERROR: {message}\n{details}")
        self._set_status("Stopped because of an error (see the log).")

        if not self._close_requested:
            messagebox.showerror("Radar error", message, parent=self.root)

    def _on_worker_finished(self, run_id, outcome):

        if run_id != self._run_id:
            return

        self._running = False
        self._worker = None
        self._update_widget_states()

        status = {
            "completed": "Finished.",
            "stopped": "Stopped.",
            "cancelled": "Cancelled.",
            "failed": "Stopped because of an error (see the log).",
        }[outcome]

        if self._last_saved_path is not None:
            status += f"\nSaved: {self._last_saved_path}"

        self._set_status(status)

        if self._plot is None and outcome != "failed":

            settings = self._current_settings

            if settings is not None and not settings.display:

                message = status.splitlines()[0]

                if self._last_saved_path is not None:
                    message += (
                        "\n\nSaved as\n" + Path(self._last_saved_path).name
                    )

                self._show_message_plot(message)

        if self._close_requested:
            self._destroy()

    # -----------------------------------------------------------------
    # Plotting (main thread only)
    # -----------------------------------------------------------------

    def _show_message_plot(self, message):

        self._plot = None

        self.figure.clear()

        axes = self.figure.add_subplot()
        axes.set_axis_off()
        axes.text(
            0.5,
            0.5,
            message,
            horizontalalignment="center",
            verticalalignment="center",
            fontsize=12,
            transform=axes.transAxes,
        )

        self.canvas.draw_idle()

    def _init_plot(self, setup):

        if setup.run_id != self._run_id:
            return

        settings = setup.settings
        range_axis_m = setup.range_axis_m
        last_range_m = float(range_axis_m[-1])

        x_min = settings.plot_min_range_m
        x_max = min(settings.plot_max_range_m, last_range_m)

        if x_max <= x_min:
            x_min, x_max = 0.0, last_range_m
            self._log(
                "The plot range lies outside the measured range "
                f"(0 - {last_range_m:.3f} m); the full range is shown."
            )

        peak_indices = np.flatnonzero(
            (range_axis_m >= settings.peak_min_range_m)
            & (range_axis_m <= settings.peak_max_range_m)
        )

        if peak_indices.size == 0:
            self._log(
                "No range bins lie in the peak-search interval; "
                "peak marking is off."
            )

        background_on = setup.background_if is not None

        # (key, title, legend): key is "beam" or the trace number.
        rows = [
            (trace, f"Trace {trace}", f"Trace {trace}")
            for trace in setup.trace_map
        ]

        if setup.beamformer is not None:
            traces_text = ", ".join(
                str(trace) for trace in setup.beamformer.trace_map
            )
            rows.insert(
                0,
                (
                    "beam",
                    f"RX beamformed - traces {traces_text} (range of trace "
                    f"{setup.beamformer.reference_trace})",
                    "Beamformed",
                ),
            )

        self.figure.clear()

        axes = self.figure.subplots(
            nrows=len(rows),
            ncols=1,
            sharex=True,
            squeeze=False,
        )[:, 0]

        empty = np.full(range_axis_m.shape, np.nan)

        artists = {}

        for axis, (key, title, legend_text) in zip(axes, rows):

            line, = axis.plot(
                range_axis_m,
                empty,
                linewidth=1.0,
                label=legend_text,
                color="tab:red" if key == "beam" else None,
                animated=True,
            )

            marker, = axis.plot(
                [],
                [],
                marker="o",
                linestyle="None",
                markersize=6,
                color="tab:orange",
                animated=True,
            )

            label = axis.text(
                0.99,
                0.94,
                "",
                transform=axis.transAxes,
                horizontalalignment="right",
                verticalalignment="top",
                clip_on=True,
                animated=True,
            )

            axis.set_title(title)
            axis.set_ylabel("Magnitude (dBFS)")
            axis.set_xlim(x_min, x_max)
            axis.set_ylim(settings.plot_min_db, settings.plot_max_db)
            axis.grid(True)

            # Animated so it is blitted on top of the curve.
            legend = axis.legend(loc="lower right")
            legend.set_animated(True)

            artists[key] = TraceArtists(line, marker, label, legend)

        axes[-1].set_xlabel("One-way range (m)")

        background_text = "ON" if background_on else "OFF"
        beamforming_text = "ON" if setup.beamformer is not None else "OFF"

        if setup.highpass is not None:
            highpass_text = f"{setup.highpass.cutoff_m:g} m"
        else:
            highpass_text = "OFF"

        self.figure.suptitle(
            f"Real-time range profiles - background subtraction "
            f"{background_text}, high-pass {highpass_text}, beamforming "
            f"{beamforming_text}"
        )

        self._plot = {
            "run_id": setup.run_id,
            "processor": RangeProcessor(
                setup.sweep_points,
                setup.background_if,
                setup.highpass,
                setup.beamformer,
            ),
            "range_axis_m": range_axis_m,
            "peak_indices": peak_indices,
            "artists": artists,
            "processing_text": (
                f"Background subtraction {background_text} | "
                f"High-pass {highpass_text} | "
                f"Beamforming {beamforming_text}"
            ),
            "full_scale": setup.full_scale,
            "background": None,
        }

        self.canvas.draw_idle()

    def _dynamic_artists(self):

        if self._plot is None:
            return []

        return [
            artist
            for trace_artists in self._plot["artists"].values()
            for artist in (
                trace_artists.line,
                trace_artists.marker,
                trace_artists.label,
                trace_artists.legend,
            )
        ]

    def _draw_dynamic_artists(self):

        for artist in self._dynamic_artists():
            artist.axes.draw_artist(artist)

    def _on_canvas_draw(self, event):
        """After every full redraw: cache the static background."""

        if (
            self._plot is None
            or event.canvas is not self.canvas
            or self.canvas.is_saving()
        ):
            return

        self._plot["background"] = self.canvas.copy_from_bbox(
            self.figure.bbox
        )

        # FigureCanvasTkAgg.draw() blits the whole buffer afterwards.
        self._draw_dynamic_artists()

    def _on_canvas_resize(self, event):

        if self._plot is not None:
            self._plot["background"] = None

    def _draw_frame(self, item):

        plot = self._plot

        if plot is None or item.run_id != plot["run_id"]:
            return

        processor = plot["processor"]
        if_data = processor.preprocess(item.frame.samples)

        # (key, complex range profile); key is "beam" or the trace number.
        curves = list(
            zip(item.frame.trace_map, processor.range_profiles(if_data))
        )

        if processor.beamformer is not None:
            curves.insert(0, ("beam", processor.beamformed_profile(if_data)))

        range_axis_m = plot["range_axis_m"]
        peak_indices = plot["peak_indices"]

        peak_messages = []

        for key, profile in curves:

            artists = plot["artists"].get(key)

            if artists is None:
                continue

            magnitude = np.abs(profile)

            # dBFS as in the 2piSENSE examples: samples divided by full
            # scale, x2 for the one-sided spectrum of the real IF signal,
            # so a full-scale sine reads 0 dBFS.
            range_db = 20.0 * np.log10(
                np.maximum(2.0 * magnitude / plot["full_scale"], 1e-15)
            )

            artists.line.set_ydata(range_db)

            if peak_indices.size == 0:
                continue

            peak_index = int(
                peak_indices[np.argmax(magnitude[peak_indices])]
            )

            peak_range_m = float(range_axis_m[peak_index])
            peak_level_db = float(range_db[peak_index])

            artists.marker.set_data([peak_range_m], [peak_level_db])
            artists.label.set_text(
                f"Strongest target peak\n"
                f"Range: {peak_range_m:.4f} m\n"
                f"Level: {peak_level_db:.1f} dBFS"
            )

            name = "BF" if key == "beam" else f"T{key}"
            peak_messages.append(f"{name}: {peak_range_m:.3f} m")

        status = [
            f"Frame {item.frame_number}",
            f"{item.rate_hz:.2f} frames/s",
            f"acquisition {item.acquisition_ms:.1f} ms",
            plot["processing_text"],
            *peak_messages,
        ]

        self.display_status_text.set(" | ".join(status))

        if plot["background"] is None:
            # No cached background yet (first frame or resize).
            self.canvas.draw_idle()
            return

        self.canvas.restore_region(plot["background"])
        self._draw_dynamic_artists()

        for trace_artists in plot["artists"].values():
            self.canvas.blit(trace_artists.line.axes.bbox)

    # -----------------------------------------------------------------
    # Worker thread (radar access only happens here)
    # -----------------------------------------------------------------

    def _run_session(self, settings, run_id):

        device = None
        link_ok = True
        outcome = "completed"

        try:

            self._post("status", "Searching for the radar...")
            self._post(
                "log",
                "Searching for the radar and applying the parameters...",
            )

            # immediate_and_receive() blocks for a whole frame.
            timeout_s = max(
                10.0,
                2.0 * settings.sweep_count * settings.sweep_period_s + 5.0,
            )

            device, config = setup_radar(
                F_START=settings.f_start_hz,
                F_STOP=settings.f_stop_hz,
                SWEEP_TIME=settings.sweep_time_s,
                SWEEP_PERIOD=settings.sweep_period_s,
                SWEEP_COUNT=settings.sweep_count,
                json_filename=str(HEALTH_REPORT_FILE),
                timeout=timeout_s,
                verbose=False,
                TRACES=list(settings.traces),
            )

            self._post("config", config)

            setup_report = read_setup_report()

            if settings.highpass:
                # Fail before the background capture if the cutoff or
                # order cannot be used; the filter itself is designed
                # from the delivered frequency axis.
                design_highpass(
                    settings.highpass_cutoff_m,
                    settings.highpass_order,
                    config.frequency_step_hz,
                    config.sweep_points,
                    config.sample_interval_s,
                )

            if (
                settings.beamforming
                and settings.bf_search_min_m >= config.max_one_sided_range_m
            ):
                raise ValueError(
                    f"The beamforming calibration window starts at "
                    f"{settings.bf_search_min_m:g} m, beyond the maximum "
                    f"range ({config.max_one_sided_range_m:.3f} m)."
                )

            if self._stop_event.is_set():
                raise AcquisitionCancelled(
                    "Stopped before the acquisition started."
                )

            background = None

            if settings.remove_background:
                background = self._capture_background(
                    device, config, settings,
                )

            elif settings.beamforming and not self._ask(
                "Beamforming calibration",
                self._calibration_instructions(settings)
                + "\n\nClick OK to calibrate the beamformer and start "
                "the acquisition.",
            ):
                raise AcquisitionCancelled("Beamforming calibration cancelled.")

            self._acquire(
                device, config, background, settings, run_id, setup_report,
            )

            if self._stop_event.is_set():
                outcome = "stopped"

        except AcquisitionCancelled as error:

            outcome = "stopped" if self._stop_event.is_set() else "cancelled"

            self._post("log", str(error))

        except Exception as error:

            link_ok = False
            outcome = "failed"

            self._post(
                "error",
                f"{type(error).__name__}: {error}",
                traceback.format_exc(),
            )

        finally:

            if device is not None:

                if link_ok:
                    self._report_error_queue(device)

                try:
                    device.close()
                    self._post("log", "Radar connection closed.")

                except Exception as error:
                    self._post(
                        "log", f"Closing the radar connection failed: {error}",
                    )

            self._post("finished", run_id, outcome)

    def _report_error_queue(self, device):

        errors = []

        try:

            error = device.check_error()

            while error is not None and len(errors) < 50:
                errors.append(
                    f"{error.code} | {error.description} | {error.info}"
                )
                error = device.check_error()

        except Exception as error:
            self._post("log", f"Reading the radar error queue failed: {error}")
            return

        if errors:
            self._post(
                "log", "Radar error queue:\n  " + "\n  ".join(errors),
            )
        else:
            self._post("log", "Radar error queue: no SCPI errors.")

    def _capture_background(self, device, config, settings):

        n_acquisitions = settings.background_acquisitions
        n_sweeps = n_acquisitions * config.sweep_count

        if not self._ask(
            "Empty-scene background",
            "Remove the corner reflector and all other test targets "
            "from the radar field of view.\n\n"
            "Do not move the radar, mounting structure, absorber, "
            "or surrounding equipment.\n\n"
            f"The radar will capture {n_acquisitions} acquisitions of "
            f"{config.sweep_count} sweep(s), for a total of "
            f"{n_sweeps} background sweeps.\n\n"
            "Click OK when the scene is empty and stable.",
        ):
            raise AcquisitionCancelled("Background capture cancelled.")

        self._post("status", "Capturing the empty-scene background...")
        self._post("log", f"Capturing the background ({n_sweeps} sweeps)...")

        started = time.perf_counter()

        background = acquire_background(
            device,
            config,
            n_acquisitions=n_acquisitions,
            verbose=False,
            progress_callback=self._make_progress_callback("Background"),
            stop_event=self._stop_event,
        )

        self._post(
            "log",
            f"Background captured from {n_sweeps} sweeps in "
            f"{time.perf_counter() - started:.2f} s.",
        )

        if settings.beamforming:
            next_step = (
                "The beamformer is calibrated next:\n\n"
                + self._calibration_instructions(settings)
                + "\n\nClick OK to calibrate the beamformer and start the "
                "acquisition."
            )
        else:
            next_step = "Click OK to start the acquisition."

        if not self._ask(
            "Background capture complete",
            "The empty-scene background has been captured.\n\n"
            "Place the target(s) in the scene now.\n\n"
            "Do not move the radar or surrounding equipment. "
            "Only place or move the targets.\n\n"
            + next_step,
        ):
            raise AcquisitionCancelled(
                "Acquisition cancelled after the background capture."
            )

        return background

    @staticmethod
    def _calibration_instructions(settings):

        return (
            "Place the corner reflector in the radar beam, where the beam "
            "should look (normally straight in front of the radar). It must "
            f"be the strongest target between {settings.bf_search_min_m:g} "
            f"and {settings.bf_search_max_m:g} m and must not move during "
            f"the calibration ({settings.bf_calibration_frames} frames).\n\n"
            "The traces are then combined coherently for targets in that "
            "direction; targets elsewhere combine less well."
        )

    def _calibrate_beamformer(
        self,
        device,
        config,
        settings,
        first_frame,
        expected_frequency_hz,
        range_axis_m,
        background_if,
        highpass,
    ):

        n_frames = settings.bf_calibration_frames

        self._post("status", "Calibrating the beamformer...")
        self._post(
            "log", f"Beamforming calibration: acquiring {n_frames} frames...",
        )

        processor = RangeProcessor(config.sweep_points, background_if, highpass)
        progress = self._make_progress_callback("Calibration")

        frame_if = []
        invalid_in_a_row = 0

        while len(frame_if) < n_frames:

            if self._stop_event.is_set():
                raise AcquisitionCancelled("Beamforming calibration stopped.")

            try:
                frame = acquire_frame(
                    device,
                    config,
                    expected_frequency_hz=expected_frequency_hz,
                )

            except InvalidFrameError as error:

                invalid_in_a_row += 1

                self._post("log", f"Calibration frame skipped: {error}")

                if invalid_in_a_row >= MAX_CONSECUTIVE_INVALID_FRAMES:
                    raise RuntimeError(
                        f"{invalid_in_a_row} consecutive invalid "
                        f"frames; last error: {error}"
                    ) from error

                continue

            invalid_in_a_row = 0

            frame_if.append(processor.preprocess(frame.samples).mean(axis=0))
            progress(len(frame_if), n_frames)

        try:
            beamformer = calibrate_beamformer(
                np.stack(frame_if),
                first_frame.trace_map,
                processor.window,
                range_axis_m,
                settings.bf_search_min_m,
                settings.bf_search_max_m,
                first_frame.full_scale or 1.0,
            )

        except ValueError as error:
            raise RuntimeError(
                f"Beamforming calibration failed: {error}"
            ) from error

        self._post("log", beamformer.describe())

        problems = beamformer.problems()

        if problems and not self._ask(
            "Weak beamforming calibration",
            "\n\n".join(problems)
            + "\n\nContinue with this calibration anyway?",
        ):
            raise AcquisitionCancelled(
                "Acquisition cancelled after the beamforming calibration."
            )

        return beamformer

    def _acquire(
        self, device, config, background, settings, run_id, setup_report,
    ):

        self._post("status", "Acquiring...")

        started = time.perf_counter()
        first_frame = acquire_frame(device, config)
        first_acquisition_ms = (time.perf_counter() - started) * 1e3

        expected_frequency_hz = first_frame.frequency_hz
        frequency_hz = first_frame.frequency_hz

        if frequency_hz is None:

            frequency_hz = np.linspace(
                config.frequency_start_hz,
                config.frequency_stop_hz,
                config.sweep_points,
            )

            self._post(
                "log",
                "The radar delivered no frequency axis; the applied "
                "start and stop frequencies are used instead.",
            )

        range_axis_m = range_axis_from_frequency(
            frequency_hz, config.sweep_points,
        )

        self._post(
            "log",
            f"Range axis: {range_axis_m.size} bins, spacing "
            f"{(range_axis_m[1] - range_axis_m[0]) * 1e3:.3f} mm, "
            f"last bin {range_axis_m[-1]:.3f} m.",
        )

        background_if = None

        if background is not None:
            background_if = background_matrix(
                background, first_frame.trace_map,
            )

        highpass = None

        if settings.highpass:

            highpass = design_highpass(
                settings.highpass_cutoff_m,
                settings.highpass_order,
                frequency_step(frequency_hz),
                config.sweep_points,
                config.sample_interval_s,
            )

            self._post("log", highpass.describe())

        beamformer = None

        if settings.beamforming:
            beamformer = self._calibrate_beamformer(
                device,
                config,
                settings,
                first_frame,
                expected_frequency_hz,
                range_axis_m,
                background_if,
                highpass,
            )

            self._post("status", "Acquiring...")

        if settings.display:

            # acquire_frame only accepts real sample types, which all
            # have a full-scale value.
            if not first_frame.full_scale:
                raise RuntimeError(
                    "The radar delivered no full-scale value for its "
                    "sample type, so levels cannot be shown in dBFS."
                )

            self._post(
                "display_init",
                DisplaySetup(
                    run_id=run_id,
                    trace_map=list(first_frame.trace_map),
                    range_axis_m=range_axis_m,
                    background_if=background_if,
                    highpass=highpass,
                    beamformer=beamformer,
                    sweep_points=config.sweep_points,
                    full_scale=first_frame.full_scale,
                    settings=settings,
                ),
            )

        recorder = None
        device_info_start = None

        if settings.save:

            # Raw samples plus the copies written at save time: float32
            # per trace, complex64 (4/n_traces of the raw size) when
            # beamformed.
            copies = (
                1
                + (2 if background is not None else 0)
                + (2 if highpass is not None else 0)
                + (4 / len(first_frame.trace_map) if beamformer else 0)
            )
            needed_bytes = (
                first_frame.samples.nbytes
                * settings.frames_to_record
                * copies
            )

            if needed_bytes > LARGE_RECORDING_BYTES and not self._ask(
                "Large recording",
                f"The recording needs about "
                f"{needed_bytes / 1024**3:.1f} GB of memory.\n\n"
                "Continue?",
            ):
                raise AcquisitionCancelled("Recording cancelled.")

            device_info_start = query_device_info(device)

            recorder = FrameRecorder(settings.frames_to_record, first_frame)

            self._post(
                "log",
                f"Recording {settings.frames_to_record} frames "
                f"({first_frame.samples.nbytes / 1024:.1f} kB each)...",
            )
            self._post(
                "progress", 0, settings.frames_to_record, "Recording",
            )

        def save(device_for_info):
            self._save_recording(
                recorder,
                config,
                background,
                highpass,
                beamformer,
                frequency_hz,
                range_axis_m,
                first_frame.full_scale,
                settings,
                setup_report,
                device_info_start,
                device_for_info,
            )

        recording_saved = False

        # After a calibration the first frame is old; start with new ones.
        pending_frame = first_frame if beamformer is None else None
        acquisition_ms = first_acquisition_ms
        frame_number = 0
        invalid_in_a_row = 0
        frame_times = deque(maxlen=RATE_HISTORY_LENGTH)
        last_progress = 0.0

        try:

            while not self._stop_event.is_set():

                if pending_frame is not None:

                    frame = pending_frame
                    pending_frame = None

                else:

                    started = time.perf_counter()

                    try:
                        frame = acquire_frame(
                            device,
                            config,
                            expected_frequency_hz=expected_frequency_hz,
                        )

                    except InvalidFrameError as error:

                        invalid_in_a_row += 1

                        self._post("log", f"Frame skipped: {error}")

                        if invalid_in_a_row >= MAX_CONSECUTIVE_INVALID_FRAMES:
                            raise RuntimeError(
                                f"{invalid_in_a_row} consecutive invalid "
                                f"frames; last error: {error}"
                            ) from error

                        continue

                    invalid_in_a_row = 0
                    acquisition_ms = (time.perf_counter() - started) * 1e3

                frame_number += 1

                now = time.perf_counter()
                frame_times.append(now)

                rate_hz = 0.0

                if len(frame_times) >= 2:
                    history_s = frame_times[-1] - frame_times[0]
                    if history_s > 0:
                        rate_hz = (len(frame_times) - 1) / history_s

                if recorder is not None and not recording_saved:

                    recorder.add(frame)

                    if (
                        recorder.full
                        or now - last_progress >= PROGRESS_INTERVAL_S
                    ):
                        last_progress = now
                        self._post(
                            "progress",
                            recorder.count,
                            recorder.n_frames,
                            "Recording",
                        )

                    if recorder.full:

                        # Set first so a failing save is not retried below.
                        recording_saved = True
                        save(device)

                        if not settings.display:
                            break

                if settings.display:
                    self._publish_display(
                        DisplayFrame(
                            run_id=run_id,
                            frame=frame,
                            frame_number=frame_number,
                            rate_hz=rate_hz,
                            acquisition_ms=acquisition_ms,
                        )
                    )

        except BaseException:

            # Keep the frames recorded so far, even after an error.
            if (
                recorder is not None
                and not recording_saved
                and recorder.count > 0
            ):
                try:
                    save(None)
                except Exception as save_error:
                    self._post(
                        "error",
                        f"Saving the partial recording failed: {save_error}",
                        traceback.format_exc(),
                    )

            raise

        if (
            recorder is not None
            and not recording_saved
            and recorder.count > 0
        ):
            save(device)

    def _save_recording(
        self,
        recorder,
        config,
        background,
        highpass,
        beamformer,
        frequency_hz,
        range_axis_m,
        full_scale,
        settings,
        setup_report,
        device_info_start,
        device,
    ):

        device_info_end = (
            query_device_info(device)
            if device is not None
            else None
        )

        measurement = recorder.to_measurement()

        self._post("status", f"Saving {measurement.n_frames} frames...")

        content = build_save_content(
            measurement=measurement,
            config=config,
            background=background,
            frequency_hz=frequency_hz,
            range_axis_m=range_axis_m,
            frame_timestamps_s=recorder.frame_timestamps_s[:recorder.count],
            trigger_timestamps_ns=(
                recorder.trigger_timestamps_ns[:recorder.count]
            ),
            full_scale=full_scale,
            n_frames_requested=recorder.n_frames,
            metadata={
                "setup_report_json": setup_report,
                "device_info_start_json": device_info_start,
                "device_info_end_json": device_info_end,
            },
            highpass=highpass,
            beamformer=beamformer,
        )

        settings.output_dir.mkdir(parents=True, exist_ok=True)

        path = unique_output_path(
            settings.output_dir, f".{settings.file_format}",
        )

        try:
            write_acquisition_file(path, content)

        except Exception as error:

            path.unlink(missing_ok=True)

            if path.suffix != ".mat":
                raise

            fallback = unique_output_path(settings.output_dir, ".npz")

            self._post(
                "log",
                f"Saving as .mat failed ({type(error).__name__}: {error}); "
                f"saving as .npz instead.",
            )

            try:
                write_acquisition_file(fallback, content)
            except Exception:
                fallback.unlink(missing_ok=True)
                raise

            path = fallback

        self._post("saved", str(path), measurement.n_frames, recorder.n_frames)


def main():

    root = tk.Tk()
    RadarGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
