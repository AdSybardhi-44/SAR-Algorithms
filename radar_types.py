# radar_types.py

"""Data containers shared by radar_setup, radar_acquisition and radar_gui."""

from dataclasses import dataclass

import numpy as np


# Speed of light in vacuum (m/s)
C = 299_792_458.0


@dataclass(frozen=True)
class RadarConfig:
    """
    Sweep configuration as applied by the radar (read back with
    SENSE:DUMP? and CALC:TRACE:LIST?). Derived quantities are properties.
    """

    frequency_start_hz: float
    frequency_stop_hz: float
    sweep_time_s: float
    # Time between the starts of the sweeps of one frame
    sweep_period_s: float
    # Sweeps per frame (per trigger)
    sweep_count: int
    # IF samples per sweep
    sweep_points: int
    sample_interval_s: float
    traces: tuple

    @property
    def bandwidth_hz(self):
        return abs(self.frequency_stop_hz - self.frequency_start_hz)

    @property
    def center_frequency_hz(self):
        return 0.5 * (self.frequency_start_hz + self.frequency_stop_hz)

    @property
    def wavelength_m(self):
        return C / self.center_frequency_hz

    @property
    def frequency_step_hz(self):
        """Frequency step between consecutive samples of a sweep."""

        return self.bandwidth_hz / (self.sweep_points - 1)

    @property
    def range_resolution_m(self):
        return C / (2.0 * self.bandwidth_hz)

    @property
    def max_range_m(self):
        """One-way range at the Nyquist frequency of the real IF signal."""

        return C / (4.0 * self.frequency_step_hz)

    def range_axis_m(self, n_bins):
        """
        One-way range of the first n_bins bins of a sweep_points-point FFT
        of a sweep:

            R_k = k * c / (2 * sweep_points * frequency_step)
        """

        return (
            np.arange(n_bins)
            * C
            / (2.0 * self.sweep_points * self.frequency_step_hz)
        )


@dataclass(frozen=True)
class RadarFrame:
    """One acquisition (one trigger) as delivered by the radar."""

    # Raw IF samples, shape (sweep_count, n_traces, sweep_points), int16
    samples: np.ndarray
    # Trace number at each index of axis 1 of samples
    trace_map: tuple
    # Start of the first sweep, POSIX seconds UTC (radar clock)
    timestamp_s: float
    # Transmit frequency of every sample of a sweep (Hz)
    frequency_hz: np.ndarray
    # Magnitude of a full-scale sample (32768 for int16)
    full_scale: float
    # The radar marked the acquisition as invalid (AcqFlags.INVALID)
    invalid: bool


@dataclass(frozen=True)
class RadarBackground:
    """Empty-scene sweep of every trace, averaged over n_frames frames."""

    # Shape (n_traces, sweep_points), float64, in trace_map order
    samples: np.ndarray
    trace_map: tuple
    n_frames: int
    # Start of the first averaged frame, POSIX seconds UTC (radar clock)
    timestamp_s: float
