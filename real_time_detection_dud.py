from collections import deque
from pathlib import Path
from pprint import pprint
import time
import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
import numpy as np
from twopilabs.sense.x1000 import SenseX1000

import numpy as np

# ---------------------------------------------------------------------
# Radar configuration
# ---------------------------------------------------------------------
F_START = 126e9
F_STOP = 182e9
# Do not set sweep_points separately.
SWEEP_TIME = 1e-3
SWEEP_PERIOD = 2e-3
# Sweeps coherently averaged in each displayed frame.
SWEEP_COUNT = 1
TRACE_LIST = [0]
C = 299_792_458.0

# ---------------------------------------------------------------------
# Background acquisition
# ---------------------------------------------------------------------
# 32 blocks x 16 sweeps = 512 background sweeps.
BACKGROUND_BLOCKS = 32
BACKGROUND_FILE = Path("realtime_background_if.npz")

# ---------------------------------------------------------------------
# Processing options
# ---------------------------------------------------------------------
# Remove the scalar ADC/DC offset from every individual sweep.
REMOVE_PER_SWEEP_DC = True
# Subtract the empty-scene IF background before the FFT.
ENABLE_BACKGROUND_SUBTRACTION = True
# Ignore immediate near-range leakage when marking the strongest target.
PEAK_SEARCH_MIN_RANGE_M = 0.1
PEAK_SEARCH_MAX_RANGE_M = 6
# The plot still displays the complete range starting from zero.
PLOT_MIN_RANGE_M = 0.0
PLOT_MAX_RANGE_M = 6
PLOT_MIN_DB = -60.0
PLOT_MAX_DB = 20.0
GUI_PAUSE_S = 0.001
RATE_HISTORY_LENGTH = 20

# ---------------------------------------------------------------------
# GUI messages
# ---------------------------------------------------------------------
def show_message(title, text):
    """
    Display a modal GUI message.
    The function returns after the user presses OK.
    """
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    root.update()
    messagebox.showinfo(
        title,
        text,
        parent=root,
    )
    root.destroy()

def ask_continue(title, text):
    """
    Display a modal confirmation window.
    Returns True if the user chooses OK, otherwise False.
    """
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    root.update()
    result = messagebox.askokcancel(
        title,
        text,
        parent=root,
    )
    root.destroy()
    return result

# ---------------------------------------------------------------------
# Radar helper functions
# ---------------------------------------------------------------------
def drain_error_queue(device):
    """Read and return all queued SCPI errors."""
    errors = []
    error = device.check_error()
    while error is not None:
        errors.append(
            {
                "code": error.code,
                "description": error.description,
                "info": error.info,
            }
        )
        error = device.check_error()
    return errors

def acquire_block(
    device,
    expected_shape,
    expected_trace_list,
    expected_frequency_hz=None,
):
    """
    Acquire and validate one complete radar block.
    The binary acquisition is read immediately before another SCPI
    command is issued.
    """
    acquisition = device.initiate.immediate_and_receive()
    if acquisition is None:
        raise RuntimeError(
            "Radar returned no acquisition object."
        )
    # Critical: read binary data immediately.
    data = acquisition.read()
    raw = np.asarray(data.array)
    frequency_hz = np.asarray(
        data.freq_axes[0],
        dtype=np.float64,
    )
    trace_map = np.asarray(
        data.trace_map,
        dtype=np.int32,
    )
    if raw.shape != expected_shape:
        raise RuntimeError(
            f"Unexpected acquisition shape {raw.shape}; "
            f"expected {expected_shape}."
        )
    if trace_map.tolist() != expected_trace_list:
        raise RuntimeError(
            f"Unexpected trace map {trace_map.tolist()}; "
            f"expected {expected_trace_list}."
        )
    if not np.all(np.isfinite(raw)):
        raise RuntimeError(
            "Acquisition contains NaN or infinity."
        )
    if not np.all(np.isfinite(frequency_hz)):
        raise RuntimeError(
            "Frequency axis contains NaN or infinity."
        )
    if not np.all(np.diff(frequency_hz) > 0):
        raise RuntimeError(
            "Frequency axis is not strictly increasing."
        )
    if expected_frequency_hz is not None:
        if not np.allclose(
            frequency_hz,
            expected_frequency_hz,
            rtol=0.0,
            atol=1.0,
        ):
            raise RuntimeError(
                "Delivered frequency axis changed."
            )
    return raw, frequency_hz, trace_map

def remove_per_sweep_dc(raw_float):
    """
    Remove the scalar mean of every sweep and trace.
    Input shape:
        sweeps x traces x frequency samples
    """
    return (
        raw_float
        - np.mean(
            raw_float,
            axis=-1,
            keepdims=True,
        )
    )

def prepare_background(background_if):
    """
    Remove the scalar mean of each background trace.
    Input shape:
        traces x frequency samples
    """
    return (
        background_if
        - np.mean(
            background_if,
            axis=-1,
            keepdims=True,
        )
    )

def process_range_profiles(
    raw,
    window,
    window_sum,
    background_if=None,
):
    """
    Processing chain:
        raw IF
            ↓
        per-sweep DC removal
            ↓
        background subtraction
            ↓
        Kaiser window
            ↓
        FFT
            ↓
        coherent averaging
    """
    raw_float = raw.astype(np.float64)
    # =====================================================
    # Remove ADC/DC offset
    # =====================================================
    if REMOVE_PER_SWEEP_DC:
        raw_float = remove_per_sweep_dc(
            raw_float
        )
    # =====================================================
    # Background subtraction on RAW IF
    # =====================================================
    if (
        ENABLE_BACKGROUND_SUBTRACTION
        and background_if is not None
    ):
        raw_float = (
            raw_float
            - background_if[None, :, :]
        )
    raw_float = raw_float - np.mean(raw_float, axis=-1, keepdims=True)
    # =====================================================
    # Apply Kaiser only after subtraction
    # =====================================================
    windowed_if = (
        raw_float
        * window[None, None, :]
    )
    # =====================================================
    # FFT
    # =====================================================
    individual_range_profiles = np.fft.rfft(
        windowed_if,
        axis=-1,
    )
    individual_range_profiles /= window_sum
    # =====================================================
    # Coherent averaging
    # =====================================================
    coherent_range_profile = np.mean(
        individual_range_profiles,
        axis=0,
    )
    return (
        coherent_range_profile,
        individual_range_profiles,
    )
# ---------------------------------------------------------------------
# Find radar
# ---------------------------------------------------------------------
devices = SenseX1000.find_devices()
if not devices:
    raise RuntimeError(
        "No X1000-series radar found."
    )

# ---------------------------------------------------------------------
# Connect and configure
# ---------------------------------------------------------------------
with SenseX1000.open_device(
    devices[0],
    timeout=15.0,
) as device:
    device.core.cls()
    # Configure full bandwidth.
    device.sense.frequency_start(F_START)
    device.sense.frequency_stop(F_STOP)
    # Sweep time controls the one-sided unambiguous range.
    device.sense.sweep_time(SWEEP_TIME)
    device.sense.sweep_period(SWEEP_PERIOD)
    device.sense.sweep_count(SWEEP_COUNT)
    device.calc.trace_list(TRACE_LIST)
    # -----------------------------------------------------------------
    # Validate coupled settings
    # -----------------------------------------------------------------
    applied_config = device.sense.dump()
    sweep_time = device.sense.sweep_time()
    sweep_period = device.sense.sweep_period()
    sweep_count = device.sense.sweep_count()
    sweep_points = device.sense.sweep_points()
    sample_interval = float(
        applied_config["SWEEP"]["TINTERVAL"]
    )
    expected_points = (
        round(sweep_time / sample_interval) + 1
    )
    expected_shape = (
        sweep_count,
        len(TRACE_LIST),
        sweep_points,
    )
    print("\n=== APPLIED CONFIGURATION ===")
    pprint(applied_config)
    if sweep_points != expected_points:
        raise RuntimeError(
            "Coupling check failed: "
            f"applied points={sweep_points}, "
            f"expected points={expected_points}."
        )
    configuration_errors = drain_error_queue(device)
    if configuration_errors:
        raise RuntimeError(
            "SCPI configuration errors: "
            f"{configuration_errors}"
        )
    # -----------------------------------------------------------------
    # Initial acquisition for frequency-axis information
    # -----------------------------------------------------------------
    print("\nAcquiring initial validation block...")
    raw_initial, frequency_hz, trace_map = acquire_block(
        device=device,
        expected_shape=expected_shape,
        expected_trace_list=TRACE_LIST,
    )
    delivered_frequency_step = float(
        np.mean(np.diff(frequency_hz))
    )
    delivered_bandwidth = float(
        frequency_hz[-1] - frequency_hz[0]
    )
    range_resolution_m = (
        C / (2.0 * delivered_bandwidth)
    )
    number_of_range_bins = (
        sweep_points // 2 + 1
    )
    range_axis_m = (
        np.arange(number_of_range_bins)
        * C
        / (
            2.0
            * sweep_points
            * delivered_frequency_step
        )
    )
    range_bin_spacing_m = float(
        range_axis_m[1] - range_axis_m[0]
    )
    print("\n=== RANGE CONFIGURATION ===")
    print(
        f"Frequency axis          : "
        f"{frequency_hz[0] / 1e9:.6f} to "
        f"{frequency_hz[-1] / 1e9:.6f} GHz"
    )
    print(
        f"Frequency step          : "
        f"{delivered_frequency_step / 1e6:.6f} MHz"
    )
    print(
        f"Range-bin spacing       : "
        f"{range_bin_spacing_m * 1e3:.3f} mm"
    )
    print(
        f"Theoretical resolution  : "
        f"{range_resolution_m * 1e3:.3f} mm"
    )
    print(
        f"Last one-sided range bin: "
        f"{range_axis_m[-1]:.3f} m"
    )
    # -----------------------------------------------------------------
    # Ask user to remove target
    # -----------------------------------------------------------------
    continue_capture = ask_continue(
        "Phase 0.2: Empty-scene background",
        (
            "Remove the corner reflector and all other test targets "
            "from the radar field of view.\n\n"
            "Do not move the radar, mounting structure, absorber, "
            "or surrounding equipment.\n\n"
            f"The radar will capture {BACKGROUND_BLOCKS} blocks of "
            f"{SWEEP_COUNT} sweeps, for a total of "
            f"{BACKGROUND_BLOCKS * SWEEP_COUNT} background sweeps.\n\n"
            "Click OK when the scene is empty and stable."
        ),
    )
    if not continue_capture:
        raise SystemExit(
            "Background capture cancelled by user."
        )
    # -----------------------------------------------------------------
    # Capture empty-scene background
    # -----------------------------------------------------------------
    background_accumulator = np.zeros(
        (
            len(TRACE_LIST),
            sweep_points,
        ),
        dtype=np.float64,
    )
    total_background_sweeps = 0
    print("\n=== CAPTURING EMPTY-SCENE BACKGROUND ===")
    background_start_time = time.perf_counter()
    for block_index in range(BACKGROUND_BLOCKS):
        (
            background_raw,
            current_frequency_hz,
            current_trace_map,
        ) = acquire_block(
            device=device,
            expected_shape=expected_shape,
            expected_trace_list=TRACE_LIST,
            expected_frequency_hz=frequency_hz,
        )
        background_float = (
            background_raw.astype(np.float64)
        )
        if REMOVE_PER_SWEEP_DC:
            background_float = remove_per_sweep_dc(
                background_float
            )
        background_accumulator += np.sum(
            background_float,
            axis=0,
        )
        total_background_sweeps += (
            background_float.shape[0]
        )
        percentage = (
            100.0
            * (block_index + 1)
            / BACKGROUND_BLOCKS
        )
        print(
            f"\rBackground capture: "
            f"{block_index + 1}/{BACKGROUND_BLOCKS} "
            f"({percentage:5.1f}%)",
            end="",
            flush=True,
        )
    background_duration_s = (
        time.perf_counter()
        - background_start_time
    )
    background_if = (
        background_accumulator
        / total_background_sweeps
    )
    # Remove any tiny residual scalar offset caused by finite averaging.
    background_if = prepare_background(
        background_if
    )
    print(
        f"\nBackground captured from "
        f"{total_background_sweeps} sweeps in "
        f"{background_duration_s:.2f} s."
    )
    # Save background for later analysis or reuse.
    np.savez_compressed(
        BACKGROUND_FILE,
        background_if=background_if,
        frequency_hz=frequency_hz,
        trace_map=trace_map,
        sweep_time_s=sweep_time,
        sweep_period_s=sweep_period,
        sweep_count=sweep_count,
        sweep_points=sweep_points,
        sample_interval_s=sample_interval,
    )
    print(
        f"Background saved to: {BACKGROUND_FILE}"
    )
    # -----------------------------------------------------------------
    # Ask user to put the target back
    # -----------------------------------------------------------------
    continue_realtime = ask_continue(
        "Background capture complete",
        (
            "The empty-scene background has been captured.\n\n"
            "Place the corner reflector in the desired range now.\n\n"
            "Do not move the radar or surrounding equipment. "
            "Only place or move the corner reflector.\n\n"
            "Click OK to start the background-subtracted "
            "real-time range display."
        ),
    )
    if not continue_realtime:
        raise SystemExit(
            "Real-time display cancelled by user."
        )
    # -----------------------------------------------------------------
    # Processing window
    # -----------------------------------------------------------------
    KAISER_BETA = 2.5
    window = np.kaiser(
        sweep_points,
        KAISER_BETA,
    )
    window_sum = float(np.sum(window))
    # -----------------------------------------------------------------
    # First target frame
    # -----------------------------------------------------------------
    target_raw, _, _ = acquire_block(
        device=device,
        expected_shape=expected_shape,
        expected_trace_list=TRACE_LIST,
        expected_frequency_hz=frequency_hz,
    )
    (
        coherent_range_complex,
        individual_range_complex,
    ) = process_range_profiles(
        raw=target_raw,
        window=window,
        window_sum=window_sum,
        background_if=background_if,
    )
    initial_magnitude = np.abs(
        coherent_range_complex
    )
    # Fixed reference from the first target frame.
    # It remains fixed instead of renormalizing every frame.
    display_reference = max(
        float(np.max(initial_magnitude)),
        1e-15,
    )
    initial_range_db = (
        20.0
        * np.log10(
            np.maximum(
                initial_magnitude / display_reference,
                1e-15,
            )
        )
    )
    # -----------------------------------------------------------------
    # Create real-time plot
    # -----------------------------------------------------------------
    plt.ion()
    figure, axes = plt.subplots(
        nrows=len(trace_map),
        ncols=1,
        figsize=(13, 9),
        sharex=True,
        constrained_layout=True,
    )
    axes = np.atleast_1d(axes)
    plot_lines = []
    peak_markers = []
    peak_labels = []
    for trace_index, trace_number in enumerate(trace_map):
        line, = axes[trace_index].plot(
            range_axis_m,
            initial_range_db[trace_index],
            linewidth=1.0,
            label=f"Trace {trace_number}",
        )
        peak_marker, = axes[trace_index].plot(
            [],
            [],
            marker="o",
            linestyle="None",
            markersize=6,
        )
        peak_label = axes[trace_index].text(
            0.99,
            0.94,
            "",
            transform=axes[trace_index].transAxes,
            horizontalalignment="right",
            verticalalignment="top",
        )
        plot_lines.append(line)
        peak_markers.append(peak_marker)
        peak_labels.append(peak_label)
        axes[trace_index].set_title(
            f"Trace {trace_number}"
        )
        axes[trace_index].set_ylabel(
            "Magnitude relative to\nfixed reference (dB)"
        )
        axes[trace_index].set_xlim(
            PLOT_MIN_RANGE_M,
            min(
                PLOT_MAX_RANGE_M,
                range_axis_m[-1],
            ),
        )
        axes[trace_index].set_ylim(
            PLOT_MIN_DB,
            PLOT_MAX_DB,
        )
        axes[trace_index].grid(True)
        axes[trace_index].legend(
            loc="lower right"
        )
    axes[-1].set_xlabel(
        "One-way range (m)"
    )
    figure.suptitle(
        "Real-time background-subtracted range profiles",
        fontsize=15,
    )
    status_text = figure.text(
        0.5,
        0.01,
        "",
        horizontalalignment="center",
    )
    plt.show(block=False)
    # -----------------------------------------------------------------
    # Target peak search region
    # -----------------------------------------------------------------
    peak_search_indices = np.flatnonzero(
        (
            range_axis_m
            >= PEAK_SEARCH_MIN_RANGE_M
        )
        & (
            range_axis_m
            <= PEAK_SEARCH_MAX_RANGE_M
        )
    )
    if peak_search_indices.size == 0:
        raise RuntimeError(
            "No range bins are available in the selected "
            "peak-search interval."
        )
    # -----------------------------------------------------------------
    # Real-time acquisition loop
    # -----------------------------------------------------------------
    frame_number = 0
    frame_times = deque(
        maxlen=RATE_HISTORY_LENGTH
    )
    print(
        "\n=== REAL-TIME DISPLAY STARTED ===\n"
        "Close the plot or press Ctrl+C to stop."
    )
    try:
        while plt.fignum_exists(figure.number):
            frame_start = time.perf_counter()
            try:
                (
                    current_raw,
                    current_frequency_hz,
                    current_trace_map,
                ) = acquire_block(
                    device=device,
                    expected_shape=expected_shape,
                    expected_trace_list=TRACE_LIST,
                    expected_frequency_hz=frequency_hz,
                )
            except RuntimeError as error:
                print(
                    f"\nFrame skipped: {error}"
                )
                continue
            # ---------------------------------------------------------
            # Background subtraction and range processing
            # ---------------------------------------------------------
            (
                coherent_range_complex,
                individual_range_complex,
            ) = process_range_profiles(
                raw=current_raw,
                window=window,
                window_sum=window_sum,
                background_if=background_if,
            )
            magnitude = np.abs(
                coherent_range_complex
            )
            # Use the fixed initial target-frame reference.
            range_db = (
                20.0
                * np.log10(
                    np.maximum(
                        magnitude / display_reference,
                        1e-15,
                    )
                )
            )
            # ---------------------------------------------------------
            # Update plots and target peak labels
            # ---------------------------------------------------------
            peak_messages = []
            for trace_index, trace_number in enumerate(
                trace_map
            ):
                plot_lines[trace_index].set_ydata(
                    range_db[trace_index]
                )
                search_values = magnitude[
                    trace_index,
                    peak_search_indices,
                ]
                local_peak_index = int(
                    np.argmax(search_values)
                )
                peak_index = int(
                    peak_search_indices[
                        local_peak_index
                    ]
                )
                peak_range_m = (float(range_axis_m[peak_index]))
                peak_level_db = float(
                    range_db[
                        trace_index,
                        peak_index,
                    ]
                )
                peak_markers[trace_index].set_data(
                    [peak_range_m],
                    [peak_level_db],
                )
                peak_labels[trace_index].set_text(
                    f"Strongest target peak\n"
                    f"Range: {peak_range_m:.4f} m\n"
                    f"Level: {peak_level_db:.1f} dB"
                )
                peak_messages.append(
                    f"T{trace_number}: "
                    f"{peak_range_m:.3f} m"
                )
            # ---------------------------------------------------------
            # Update frame-rate information
            # ---------------------------------------------------------
            frame_number += 1
            frame_end = time.perf_counter()
            frame_times.append(frame_end)
            if len(frame_times) >= 2:
                history_duration = (
                    frame_times[-1]
                    - frame_times[0]
                )
                update_rate_hz = (
                    (len(frame_times) - 1)
                    / history_duration
                    if history_duration > 0
                    else 0.0
                )
            else:
                update_rate_hz = 0.0
            frame_duration_ms = (
                (frame_end - frame_start) * 1e3
            )
            status_text.set_text(
                f"Frame {frame_number} | "
                f"{update_rate_hz:.2f} updates/s | "
                f"{frame_duration_ms:.1f} ms | "
                "Background subtraction ON | "
                + " | ".join(peak_messages)
            )
            figure.canvas.draw_idle()
            figure.canvas.flush_events()
            plt.pause(GUI_PAUSE_S)
    except KeyboardInterrupt:
        print(
            "\nReal-time display stopped by user."
        )
    finally:
        plt.ioff()
        if plt.fignum_exists(figure.number):
            plt.close(figure)
        final_errors = drain_error_queue(device)
        print("\n=== FINAL ERROR QUEUE ===")
        if final_errors:
            for error in final_errors:
                print(error)
        else:
            print("No SCPI error")
        print("Radar connection closed.")