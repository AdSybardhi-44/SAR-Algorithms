from datetime import datetime, timezone

import numpy as np

from radar_types import RadarFrame, RadarMeasurement


class InvalidFrameError(RuntimeError):
    """
    A complete acquisition was received but failed validation.

    The data was read completely, so the radar is ready for
    the next acquisition.
    """


class AcquisitionCancelled(Exception):
    """Raised when an acquisition is stopped through stop_event."""


def acquire_frame(
    device,
    config=None,
    expected_frequency_hz=None,
):
    """
    Acquire one frame (one trigger of sweep_count sweeps).

    Parameters
    ----------
    device
        Configured X1000 device.

    config : RadarConfig, optional
        If given, the delivered shape and traces are
        checked against it.

    expected_frequency_hz : np.ndarray, optional
        If given, the delivered frequency axis must match
        it within 1 Hz.

    Returns
    -------
    RadarFrame

    Data shape:

        (sweep_count,
         n_traces,
         sweep_points)

    Raises
    ------
    InvalidFrameError
        The acquisition was read completely but is unusable.
    """

    acquisition = device.initiate.immediate_and_receive()

    if acquisition is None:
        raise InvalidFrameError(
            "Radar returned no acquisition."
        )

    #
    # The binary data must be read before any other
    # SCPI command is sent to the radar.
    #

    data = acquisition.read()

    timestamp_utc = (
        datetime.now(
            timezone.utc
        ).isoformat()
    )

    #
    # data.array is a read-only view of the receive buffer.
    #

    samples = np.array(data.array)

    trace_map = [
        int(trace_id)
        for trace_id in data.trace_map
    ]

    if samples.dtype.kind not in "iuf":
        raise InvalidFrameError(
            f"Unsupported sample type "
            f"{data.header.acq_dtype.name}; "
            f"expected real IF samples."
        )

    if (
        samples.ndim != 3
        or samples.shape[1] != len(trace_map)
    ):
        raise InvalidFrameError(
            f"Unexpected acquisition shape "
            f"{samples.shape} for traces {trace_map}."
        )

    if config is not None:

        expected_shape = (
            config.sweep_count,
            len(config.enabled_traces),
            config.sweep_points,
        )

        if samples.shape != expected_shape:
            raise InvalidFrameError(
                f"Unexpected acquisition shape "
                f"{samples.shape}; expected "
                f"{expected_shape}."
            )

        expected_traces = sorted(
            int(trace_id)
            for trace_id in config.enabled_traces
        )

        if sorted(trace_map) != expected_traces:
            raise InvalidFrameError(
                f"Unexpected trace map {trace_map}; "
                f"expected {expected_traces}."
            )

    if (
        samples.dtype.kind == "f"
        and not np.all(np.isfinite(samples))
    ):
        raise InvalidFrameError(
            "Acquisition contains NaN or infinity."
        )

    frequency_hz = None

    if (
        len(data.freq_axes) > 0
        and data.freq_axes[0] is not None
    ):

        frequency_hz = np.array(
            data.freq_axes[0],
            dtype=np.float64,
        )

        frequency_steps = np.diff(frequency_hz)

        if (
            frequency_hz.shape != (samples.shape[-1],)
            or not np.all(np.isfinite(frequency_hz))
            or not (
                np.all(frequency_steps > 0)
                or np.all(frequency_steps < 0)
            )
        ):
            raise InvalidFrameError(
                "Delivered frequency axis is invalid."
            )

    if expected_frequency_hz is not None:

        if (
            frequency_hz is None
            or frequency_hz.shape
            != np.shape(expected_frequency_hz)
            or not np.allclose(
                frequency_hz,
                expected_frequency_hz,
                rtol=0.0,
                atol=1.0,
            )
        ):
            raise InvalidFrameError(
                "Delivered frequency axis changed."
            )

    trigger_timestamp_ns = None

    if len(data.timestamps) > 0:

        first_timestamp = np.datetime64(
            data.timestamps[0],
            "ns",
        )

        if not np.isnat(first_timestamp):
            trigger_timestamp_ns = int(
                first_timestamp.astype(np.int64)
            )

    full_scale = data.header.acq_dtype.info.mag

    return RadarFrame(

        timestamp_utc=timestamp_utc,

        samples=samples,

        trace_map=trace_map,

        frequency_hz=frequency_hz,

        full_scale=(
            float(full_scale)
            if full_scale is not None
            else None
        ),

        trigger_timestamp_ns=trigger_timestamp_ns,
    )


def acquire_sweeps(
    device,
    config=None,
    n_frames=1,
    verbose=False,
):
    """
    Acquire one or more radar frames.

    Parameters
    ----------
    device
        Configured X1000 device.

    config : RadarConfig, optional

    n_frames : int
        Number of frames to acquire.

    verbose : bool

    Returns
    -------
    RadarMeasurement

    Data shape:

        (n_frames,
         sweep_count,
         sweep_points)
    """

    if n_frames < 1:
        raise ValueError(
            "n_frames must be >= 1."
        )

    timestamp_utc = (
        datetime.now(
            timezone.utc
        ).isoformat()
    )

    traces = device.calc.trace_list()

    if config is not None:

        sweep_count = config.sweep_count
        sweep_points = config.sweep_points

    else:

        sweep_count = device.sense.sweep_count()
        sweep_points = device.sense.sweep_points()

    rx0_frames = []
    rx1_frames = []
    rx2_frames = []

    # ==========================================
    # Acquire frames
    # ==========================================

    for frame_idx in range(n_frames):

        if verbose:
            print(
                f"Acquiring frame "
                f"{frame_idx + 1}/{n_frames}"
            )

        frame = acquire_frame(device, config)

        for trace_index, trace_id in enumerate(
            frame.trace_map
        ):

            #
            # Shape:
            # (sweep_count, sweep_points)
            #

            data = frame.samples[:, trace_index, :]

            if trace_id == 0:
                rx0_frames.append(data)

            elif trace_id == 1:
                rx1_frames.append(data)

            elif trace_id == 2:
                rx2_frames.append(data)

    # ==========================================
    # Stack into
    # (n_frames, sweep_count, sweep_points)
    # ==========================================

    rx0 = (
        np.stack(rx0_frames, axis=0)
        if len(rx0_frames) > 0
        else None
    )

    rx1 = (
        np.stack(rx1_frames, axis=0)
        if len(rx1_frames) > 0
        else None
    )

    rx2 = (
        np.stack(rx2_frames, axis=0)
        if len(rx2_frames) > 0
        else None
    )

    measurement = RadarMeasurement(

        timestamp_utc=timestamp_utc,

        n_frames=n_frames,

        sweep_count=sweep_count,
        sweep_points=sweep_points,

        traces=traces,

        rx0=rx0,
        rx1=rx1,
        rx2=rx2,
    )

    if verbose:

        print("\nAcquisition complete")

        if rx0 is not None:
            print(
                f"RX0 shape: {rx0.shape}"
            )

        if rx1 is not None:
            print(
                f"RX1 shape: {rx1.shape}"
            )

        if rx2 is not None:
            print(
                f"RX2 shape: {rx2.shape}"
            )

    return measurement

from datetime import datetime, timezone

import numpy as np

from radar_types import RadarBackground


def acquire_background(
    device,
    config,
    n_acquisitions=100,
    verbose=True,
    progress_callback=None,
    stop_event=None,
):
    """
    Acquire an averaged background measurement.

    Parameters
    ----------
    device
        Configured radar.

    config : RadarConfig

    n_acquisitions : int
        Number of acquisitions to average.

    progress_callback : callable, optional
        Called as progress_callback(completed, total)
        after every acquisition.

    stop_event : threading.Event, optional
        If set, the acquisition stops and
        AcquisitionCancelled is raised.

    Returns
    -------
    RadarBackground
    """

    if n_acquisitions < 1:
        raise ValueError(
            "n_acquisitions must be >= 1."
        )

    timestamp_utc = (
        datetime.now(
            timezone.utc
        ).isoformat()
    )

    traces = device.calc.trace_list()

    rx0_all = []
    rx1_all = []
    rx2_all = []

    if verbose:

        print("\n=== BACKGROUND ACQUISITION ===")
        print(
            f"Acquisitions: {n_acquisitions}"
        )

    # ==================================
    # Collect measurements
    # ==================================

    for acquisition_idx in range(
        n_acquisitions
    ):

        if (
            stop_event is not None
            and stop_event.is_set()
        ):
            raise AcquisitionCancelled(
                "Background acquisition stopped."
            )

        if verbose:

            print(
                f"\rAcquisition "
                f"{acquisition_idx + 1}"
                f"/{n_acquisitions}",
                end="",
                flush=True,
            )

        frame = acquire_frame(device, config)

        for trace_index, trace_id in enumerate(
            frame.trace_map
        ):

            data = frame.samples[:, trace_index, :]

            if trace_id == 0:
                rx0_all.append(data)

            elif trace_id == 1:
                rx1_all.append(data)

            elif trace_id == 2:
                rx2_all.append(data)

        if progress_callback is not None:
            progress_callback(
                acquisition_idx + 1,
                n_acquisitions,
            )

    if verbose:
        print("\nBackground acquisition complete.")

    # ==================================
    # Convert to arrays
    # ==================================

    rx0_raw = (
        np.stack(rx0_all, axis=0)
        if len(rx0_all) > 0
        else None
    )

    rx1_raw = (
        np.stack(rx1_all, axis=0)
        if len(rx1_all) > 0
        else None
    )

    rx2_raw = (
        np.stack(rx2_all, axis=0)
        if len(rx2_all) > 0
        else None
    )

    # ==================================
    # Average all acquisitions
    # ==================================

    rx0_background = None
    rx1_background = None
    rx2_background = None

    if rx0_raw is not None:

        rx0_background = np.mean(
            rx0_raw,
            axis=(0, 1)
        )

    if rx1_raw is not None:

        rx1_background = np.mean(
            rx1_raw,
            axis=(0, 1)
        )

    if rx2_raw is not None:

        rx2_background = np.mean(
            rx2_raw,
            axis=(0, 1)
        )

    return RadarBackground(

        timestamp_utc=timestamp_utc,

        n_acquisitions=n_acquisitions,

        sweep_count=config.sweep_count,
        sweep_points=config.sweep_points,

        traces=traces,

        rx0_raw=rx0_raw,
        rx1_raw=rx1_raw,
        rx2_raw=rx2_raw,

        rx0_background=rx0_background,
        rx1_background=rx1_background,
        rx2_background=rx2_background,
    )