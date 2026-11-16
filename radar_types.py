from dataclasses import dataclass
import numpy as np


@dataclass
class RadarConfig:

    frequency_start_hz: float
    frequency_stop_hz: float

    bandwidth_hz: float

    sweep_time_s: float
    sweep_period_s: float
    sweep_count: int

    sweep_points: int

    sample_interval_s: float
    effective_sample_rate_hz: float

    frequency_step_hz: float

    range_resolution_m: float
    max_one_sided_range_m: float

    expected_points: int
    timing_consistent: bool

    center_frequency_hz: float
    wavelength_m: float
    chirp_slope_hz_per_s: float

    range_axis_m: np.ndarray

    enabled_traces: list

    setup_timestamp_utc: str

    main_pll_locked: bool
    aux_pll_locked: bool

    radar_state: str

    ready_for_acquisition: bool

@dataclass
class RadarFrame:

    timestamp_utc: str

    #
    # Raw samples of one acquisition (one trigger)
    # Shape:
    # (sweep_count, n_traces, sweep_points)
    #
    samples: np.ndarray

    #
    # Trace number stored at each index of axis 1
    #
    trace_map: list

    frequency_hz: np.ndarray | None = None

    full_scale: float | None = None

    trigger_timestamp_ns: int | None = None

@dataclass
class RadarMeasurement:

    timestamp_utc: str

    n_frames: int

    sweep_count: int
    sweep_points: int

    traces: list

    #
    # Shape:
    # (n_frames, sweep_count, sweep_points)
    #
    rx0: np.ndarray | None = None
    rx1: np.ndarray | None = None
    rx2: np.ndarray | None = None

@dataclass
class RadarBackground:

    timestamp_utc: str

    n_acquisitions: int

    sweep_count: int
    sweep_points: int

    traces: list

    #
    # Raw acquisitions
    #
    rx0_raw: np.ndarray | None = None
    rx1_raw: np.ndarray | None = None
    rx2_raw: np.ndarray | None = None

    #
    # Averaged background
    # Shape:
    # (sweep_points,)
    #
    rx0_background: np.ndarray | None = None
    rx1_background: np.ndarray | None = None
    rx2_background: np.ndarray | None = None