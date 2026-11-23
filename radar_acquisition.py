# radar_acquisition.py

"""
Acquisition, background capture and recording.

Recording files
---------------
Recording.close() writes a .mat (MATLAB) or .npz (NumPy) file with the
raw data exactly as the radar delivered them and what is needed to
process them. Sizes are MATLAB sizes; NumPy loads the same shapes
(radar_signal.T is (N_az, N_rx, N_samples)). Counts are stored as double.

    radar_signal                  int16 [N_samples x N_rx x N_az] raw IF
                                  samples; N_az = every recorded sweep in
                                  time order (frames x sweeps per frame)
    radar_rx                      [1 x N_rx] trace number of every index of
                                  dimension 2 (0 = TRx, 1 and 2 = Rx)
    radar_rx_number               N_rx
    radar_frames                  N_az
    radar_sweeps_per_frame        sweeps per frame (per trigger)
    radar_timestamps              [1 x N_az] start of every sweep, POSIX
                                  seconds UTC from the radar clock, which is
                                  set to the PC clock when it is configured
                                  and again when the recording starts: the
                                  time base of the stage position log of
                                  actuator_logger.m (see actuator_log.py)
    radar_carrier_freq            Hz, centre of the sweep
    radar_carrier_wavelength      m
    radar_bandwidth               Hz
    radar_sweep_time              s
    radar_sweep_period            s, between the sweep starts of a frame
    radar_sample_interval         s, between IF samples
    radar_frequency_axis          [1 x N_samples] Hz, transmit frequency of
                                  every sample of a sweep
    radar_range_samples_number    N_samples
    radar_range_samples           [1 x N_samples] m, one-way range of bin k
                                  of an N_samples-point FFT of a sweep,
                                  k = 0 ... N_samples-1; bins
                                  0 ... floor(N_samples/2) are unique
                                  because the IF samples are real
    radar_range_resolution        m, c / (2 B)
    radar_lightspeed_coefficient  m/s, c
    radar_full_scale              magnitude of a full-scale sample (32768)
    radar_background              [N_samples x N_rx] mean empty-scene sweep;
                                  empty when no background was captured
    radar_background_frames       frames averaged into radar_background
    radar_device                  identity of the radar (*IDN?: maker,
                                  model, serial number, firmware)
"""

from pathlib import Path

import numpy as np
from scipy.io import savemat
from twopilabs.sense.x1000 import SenseX1000

from radar_types import C, RadarBackground, RadarFrame


# Largest variable that a MATLAB v7 .mat file can hold
MAT_MAX_VARIABLE_BYTES = 2**31


class AcquisitionCancelled(Exception):
    """Raised when an acquisition is stopped through stop_event."""


def acquire_frame(device):
    """
    Arm, trigger and read one acquisition: sweep_count sweeps of every
    enabled trace. The library raises the errors the radar reports.
    """

    acquisition = device.initiate.immediate_and_receive()

    if acquisition is None:
        raise RuntimeError("The radar returned no acquisition.")

    # Read all data before sending any other command to the radar.
    data = acquisition.read()

    return RadarFrame(
        # data.array is a read-only view of the receive buffer.
        samples=np.array(data.array),
        trace_map=tuple(int(trace) for trace in data.trace_map),
        timestamp_s=int(data.timestamps[0].astype(np.int64)) * 1e-9,
        frequency_hz=np.array(data.freq_axes[0], dtype=np.float64),
        full_scale=float(data.header.acq_dtype.info.mag),
        invalid=bool(data.header.acq_flags & SenseX1000.AcqFlags.INVALID),
    )


class BackgroundAverager:
    """Averages every sweep of n_frames consecutive frames, per trace."""

    def __init__(self, n_frames):

        self.n_frames = n_frames
        self.count = 0

        self._first = None
        self._sum = None

    def add(self, frame):
        """Add a frame; returns True once n_frames frames were added."""

        sweep_sum = frame.samples.sum(axis=0, dtype=np.float64)

        if self._first is None:
            self._first = frame
            self._sum = sweep_sum
        else:
            self._sum += sweep_sum

        self.count += 1

        return self.count >= self.n_frames

    def result(self):

        return RadarBackground(
            samples=self._sum / (self.count * self._first.samples.shape[0]),
            trace_map=self._first.trace_map,
            n_frames=self.count,
            timestamp_s=self._first.timestamp_s,
        )


def capture_background(
    device,
    n_frames,
    progress_callback=None,
    stop_event=None,
):
    """
    Average the next n_frames frames (of an empty scene).

    progress_callback, if given, is called as progress_callback(done,
    n_frames) after every frame. Setting stop_event raises
    AcquisitionCancelled.
    """

    averager = BackgroundAverager(n_frames)
    done = False

    while not done:

        if stop_event is not None and stop_event.is_set():
            raise AcquisitionCancelled("Background capture stopped.")

        done = averager.add(acquire_frame(device))

        if progress_callback is not None:
            progress_callback(averager.count, n_frames)

    return averager.result()


def write_recording_file(path, content):
    """Write content compressed as .mat (MATLAB) or .npz (NumPy)."""

    if path.suffix == ".mat":
        savemat(str(path), content, do_compression=True, oned_as="row")
    else:
        np.savez_compressed(path, **content)


class Recording:
    """
    Records frames as they arrive and writes the recording file described
    in the module docstring on close().

    The samples are appended to a .part file next to the recording file,
    so the memory use does not grow with the recording length; close()
    deletes it once the recording file is written. A .mat recording
    larger than MAT_MAX_VARIABLE_BYTES is written as .npz.
    """

    def __init__(self, path, config, device_id="", background=None):

        self.path = Path(path)
        self.part_path = self.path.with_suffix(".part")
        self.config = config
        self.device_id = device_id
        self.background = background

        # Messages for the user about how the file was saved
        self.notes = []

        self.n_frames = 0
        self.n_bytes = 0

        self._first = None
        self._timestamps = []
        self._file = open(self.part_path, "xb")

    @property
    def duration_s(self):
        """Time from the first to the last recorded frame."""

        if self.n_frames < 2:
            return 0.0

        return self._timestamps[-1] - self._timestamps[0]

    @property
    def sweep_time_range_s(self):
        """
        (start of the first sweep, end of the last sweep) in POSIX seconds
        UTC, the clock of radar_timestamps; None before the first frame.
        """

        if self._first is None:
            return None

        sweep_count = self._first.samples.shape[0]

        last_start = (
            self._timestamps[-1]
            + (sweep_count - 1) * self.config.sweep_period_s
        )

        return self._timestamps[0], last_start + self.config.sweep_time_s

    def add_frame(self, frame):

        if self._first is None:
            self._first = frame

        self._file.write(frame.samples.tobytes())
        self._timestamps.append(frame.timestamp_s)

        self.n_frames += 1
        self.n_bytes += frame.samples.nbytes

    def describe_part(self):
        """Layout of the .part file, to read it if saving failed."""

        if self._first is None:
            return f"{self.part_path} (no frames)"

        sweep_count, n_traces, n_points = self._first.samples.shape

        return (
            f"{self.part_path}: raw {self._first.samples.dtype} samples in "
            f"C order, shape ({self.n_frames * sweep_count}, {n_traces}, "
            f"{n_points}) = (sweeps, traces {list(self._first.trace_map)}, "
            f"samples)"
        )

    def close(self):
        """
        Write the recording file and delete the .part file. Returns the
        path of the written file, or None if no frame was recorded. If
        writing fails, the .part file is kept (see describe_part()).
        """

        self._file.close()

        if self._first is None:
            self.part_path.unlink()
            return None

        sweep_count, n_traces, n_points = self._first.samples.shape

        path = self.path

        if path.suffix == ".mat" and self.n_bytes >= MAT_MAX_VARIABLE_BYTES:

            path = path.with_suffix(".npz")

            self.notes.append(
                f"The recording is larger than the 2 GiB that MATLAB can "
                f"store in one .mat variable; it is saved as {path.name} "
                f"instead."
            )

        samples = np.memmap(
            self.part_path,
            dtype=self._first.samples.dtype,
            mode="r",
            shape=(self.n_frames * sweep_count, n_traces, n_points),
        )

        # The .part file holds the sweeps one after the other, so the
        # transpose is the (N_samples, N_rx, N_az) signal without a copy.
        path = self._write(path, samples.T)

        # Windows cannot delete a file that is still memory-mapped.
        del samples

        self.part_path.unlink()

        return path

    def _write(self, path, signal):

        content = self._content(signal)

        try:
            write_recording_file(path, content)
            return path

        except Exception as error:

            path.unlink(missing_ok=True)

            if path.suffix != ".mat":
                raise

            fallback = path.with_suffix(".npz")

            self.notes.append(
                f"Saving as .mat failed ({type(error).__name__}: {error}); "
                f"the recording is saved as {fallback.name} instead."
            )

            try:
                write_recording_file(fallback, content)
            except Exception:
                fallback.unlink(missing_ok=True)
                raise

            return fallback

    def _content(self, signal):

        config = self.config
        first = self._first
        sweep_count, n_traces, n_points = first.samples.shape

        timestamps = (
            np.asarray(self._timestamps)[:, None]
            + np.arange(sweep_count) * config.sweep_period_s
        ).ravel()

        if self.background is None:
            background = np.zeros((0, 0))
            background_frames = 0
        else:
            background = self.background.samples.T
            background_frames = self.background.n_frames

        return {
            "radar_signal": signal,
            "radar_rx": np.asarray(first.trace_map, dtype=np.float64),
            "radar_rx_number": float(n_traces),
            "radar_frames": float(signal.shape[2]),
            "radar_sweeps_per_frame": float(sweep_count),
            "radar_timestamps": timestamps,
            "radar_carrier_freq": config.center_frequency_hz,
            "radar_carrier_wavelength": config.wavelength_m,
            "radar_bandwidth": config.bandwidth_hz,
            "radar_sweep_time": config.sweep_time_s,
            "radar_sweep_period": config.sweep_period_s,
            "radar_sample_interval": config.sample_interval_s,
            "radar_frequency_axis": first.frequency_hz,
            "radar_range_samples_number": float(n_points),
            "radar_range_samples": config.range_axis_m(n_points),
            "radar_range_resolution": config.range_resolution_m,
            "radar_lightspeed_coefficient": C,
            "radar_full_scale": first.full_scale,
            "radar_background": background,
            "radar_background_frames": float(background_frames),
            "radar_device": self.device_id,
        }
