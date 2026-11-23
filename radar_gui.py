# radar_gui.py

"""
Graphical user interface for the X1000 radar.

    1. Set the radar parameters and the recording options.
    2. Choose a mode and press Start:

       Real-time display  A second window shows the range profiles of the
                          traces ticked there. Background subtraction and
                          the coupling high-pass filter change only the
                          display and can be switched at any time without
                          reconnecting the radar. Record saves the raw
                          frames, as "Save data" does, while the display
                          continues.

       Save data          Raw frames are recorded until Stop is pressed
                          (or for a fixed number of frames). Only the
                          progress, the recording length and the data size
                          are shown.

Recordings always hold the raw IF samples as the radar delivers them
(int16), never the display processing. radar_acquisition.py lists the
contents of the recording files.

Stage positions (scans with the linear stage of OTA Hub): actuator_logger.m,
run in the MATLAB of the Linear Actuator App, logs the stage position with
PC time stamps (actuator_log.py describes the file). Every recording sets
the radar clock to the PC clock when it starts, so radar_timestamps and the
stage log share one time base. The log says when a recording starts
whether the stage is being logged, and after saving which stage log covers
the recording.

The display shows absolute levels in dBFS, as in the 2piSENSE examples:
20*log10(2*|rfft(x*w)/sum(w)| / full_scale), so a full-scale sine reads
0 dBFS.

Run from the repository folder:

    .venv\\Scripts\\python.exe radar_gui.py
"""

import ctypes
import math
import queue
import shutil
import sys
import tempfile
import threading
import time
import tkinter as tk
import traceback
from collections import deque
from dataclasses import dataclass, field
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
from scipy.signal import butter, sosfiltfilt, zpk2sos

from actuator_log import logger_status, recording_report
from radar_acquisition import (
    AcquisitionCancelled,
    BackgroundAverager,
    Recording,
    acquire_frame,
    capture_background,
)
from radar_setup import setup_radar, sync_radar_clock
from radar_types import RadarConfig, RadarFrame


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "Results"
HEALTH_REPORT_FILE = SCRIPT_DIR / "radar_health.json"

# Same window as real_time_detection_dud.py
KAISER_BETA = 2.5

# filtfilt pads each sweep until the high-pass impulse response has
# decayed to this level, so its start-up transient never reaches the data.
HIGHPASS_PADDING_DECAY = 1e-6

POLL_INTERVAL_MS = 20
MAX_EVENTS_PER_POLL = 200
MAX_LOG_LINES = 2000
RATE_HISTORY_LENGTH = 20
STATUS_INTERVAL_S = 0.2

# A recording stops when the free disk space no longer holds the
# recording file plus this margin.
DISK_CHECK_INTERVAL_S = 2.0
MIN_FREE_DISK_BYTES = 1024**3

# Sizes in pixels of a 100 % (96 dpi) display, scaled with the display
# DPI. Windows open at least this large, maximized when that does not fit.
MAIN_WINDOW_SIZE = (1100, 760)
DISPLAY_WINDOW_SIZE = (1400, 900)
MIN_LOG_WIDTH = 300
MIN_PLOT_WIDTH = 360
MIN_WINDOW_HEIGHT = 440
CONTROL_TEXT_WIDTH = 280
# Control panel scrolling: pixels per step, steps per mouse-wheel notch.
SCROLL_STEP = 20
WHEEL_STEPS = 3
# Plot rows lower than this (in points) hide their titles and show the
# curve name and peak on one line, so that short screens keep a readable
# plot.
PLOT_COMPACT_ROW_HEIGHT_PT = 100

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
    "frames_to_record": "1000",
    "background_frames": "100",
}

DISPLAY_DEFAULT_VALUES = {
    "background_frames": "100",
    # Measured on this radar: its own near-range clutter reaches ~20 cm.
    "highpass_cutoff_m": "0.2",
    "highpass_order": "6",
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

    # Real-time display (True) or save data only (False)
    display: bool

    output_dir: Path
    file_format: str
    # 0 = until Stop
    frames_to_record: int
    # Save mode: frames of an empty-scene background captured before the
    # recording (0 = none)
    background_frames: int


@dataclass(frozen=True)
class DisplaySetup:

    run_id: int
    config: RadarConfig
    trace_map: tuple
    full_scale: float
    settings: RunSettings


@dataclass(frozen=True)
class DisplayFrame:

    run_id: int
    frame: RadarFrame
    frame_number: int
    rate_hz: float
    acquisition_ms: float


@dataclass(frozen=True)
class RecordingStatus:

    n_frames: int
    # 0 = until Stop
    frames_limit: int
    duration_s: float
    n_bytes: int


@dataclass
class Session:
    """State of one radar session (worker thread only)."""

    device: object
    config: RadarConfig
    settings: RunSettings
    run_id: int
    device_id: str
    invalid_frames: int = 0
    # Threads writing finished recordings; the radar keeps running.
    savers: list = field(default_factory=list)


@dataclass
class ActiveRecording:
    """A Recording with the bookkeeping of the worker loop."""

    recording: Recording
    frames_limit: int
    # time.monotonic() values
    next_status: float = 0.0
    next_disk_check: float = 0.0


@dataclass
class TraceArtists:

    line: object
    marker: object
    label: object
    legend: object
    # Last (range_m, level_dbfs) shown by the peak label.
    peak: tuple | None = None


# =====================================================================
# Helpers
# =====================================================================

def parse_number(text, label):

    try:
        value = float(text.strip().replace(",", "."))
    except ValueError:
        raise ValueError(f"{label}: '{text}' is not a number.") from None

    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number.")

    return value


def parse_integer(text, label, minimum=None):

    try:
        value = int(text.strip())
    except ValueError:
        raise ValueError(f"{label}: '{text}' is not a whole number.") from None

    if minimum is not None and value < minimum:
        raise ValueError(f"{label} must be at least {minimum}.")

    return value


def parse_interval(low_text, high_text, label):

    low = parse_number(low_text, f"{label} (from)")
    high = parse_number(high_text, f"{label} (to)")

    if high <= low:
        raise ValueError(f"{label}: 'to' must be greater than 'from'.")

    return low, high


def format_duration(seconds):
    """mm:ss.s, or h:mm:ss.s from one hour on."""

    tenths = round(max(seconds, 0.0) * 10)
    minutes, tenths = divmod(tenths, 600)
    hours, minutes = divmod(minutes, 60)
    text = f"{minutes:02d}:{tenths // 10:02d}.{tenths % 10}"

    return f"{hours}:{text}" if hours else text


def format_bytes(n_bytes):
    """Size as Windows Explorer shows it (1 KB = 1024 bytes)."""

    for unit, size in (("GB", 1024**3), ("MB", 1024**2), ("KB", 1024)):
        if n_bytes >= size:
            return f"{n_bytes / size:.1f} {unit}"

    return f"{n_bytes} bytes"


def recording_status_text(status):

    if status.frames_limit:
        frames = f"{status.n_frames} of {status.frames_limit} frames"
    else:
        frames = f"{status.n_frames} frames"

    rate_hz = 0.0

    if status.duration_s > 0.0:
        rate_hz = (status.n_frames - 1) / status.duration_s

    return (
        f"Recording {format_duration(status.duration_s)} | {frames} | "
        f"{format_bytes(status.n_bytes)} raw | {rate_hz:.0f} frames/s"
    )


def unique_output_path(folder, suffix):

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    index = 0

    while True:

        name = f"acquisition_{stamp}" + (f"_{index}" if index else "")
        candidate = folder / f"{name}{suffix}"

        if not any(
            candidate.with_suffix(other).exists()
            for other in (".mat", ".npz", ".part")
        ):
            return candidate

        index += 1


# =====================================================================
# Display processing (GUI thread)
# =====================================================================

@dataclass(frozen=True)
class HighpassFilter:
    """Zero-phase Butterworth high-pass along each sweep."""

    cutoff_m: float
    order: int
    sos: np.ndarray
    padlen: int

    def apply(self, if_data):

        return sosfiltfilt(
            self.sos,
            if_data,
            axis=-1,
            padtype="odd",
            padlen=self.padlen,
        )


def design_highpass(cutoff_m, order, config):
    """
    Butterworth high-pass that suppresses everything closer than
    cutoff_m (the antenna coupling). Range maps linearly to IF frequency,
    so the normalized cutoff is cutoff_m / max_range (1 = Nyquist).
    """

    if not 0.0 < cutoff_m < config.max_range_m:
        raise ValueError(
            f"The cutoff range must lie between 0 and the maximum range "
            f"({config.max_range_m:.3f} m)."
        )

    zeros, poles, gain = butter(
        order,
        cutoff_m / config.max_range_m,
        btype="highpass",
        output="zpk",
    )

    pole_radius = min(max(float(np.max(np.abs(poles))), 1e-12), 1.0 - 1e-12)

    decay_samples = math.ceil(
        math.log(HIGHPASS_PADDING_DECAY) / math.log(pole_radius)
    )

    return HighpassFilter(
        cutoff_m=cutoff_m,
        order=order,
        sos=zpk2sos(zeros, poles, gain),
        # filtfilt needs padlen < sweep_points - 1.
        padlen=max(0, min(decay_samples, config.sweep_points - 2)),
    )


class RangeProcessor:
    """
    Range profiles of the display:

        raw IF
            -> background subtraction (optional)
            -> per-sweep DC removal
            -> high-pass filter with filtfilt (optional)
            -> Kaiser window
            -> FFT
            -> coherent average over the sweeps of a frame
            -> dBFS

    background and highpass can be replaced at any time.
    """

    def __init__(self, sweep_points, full_scale):

        self.window = np.kaiser(sweep_points, KAISER_BETA)

        # dBFS as in the 2piSENSE examples: samples divided by full scale,
        # x2 for the one-sided spectrum of the real IF signal, so a
        # full-scale sine reads 0 dBFS.
        self.scale = 2.0 / (float(np.sum(self.window)) * full_scale)

        # (n_traces, sweep_points) in trace_map order, or None
        self.background = None
        self.highpass = None

    def profiles_db(self, samples, indices):
        """
        samples: (sweep_count, n_traces, sweep_points); indices: trace
        positions to process. Returns (len(indices), n_bins) in dBFS.
        """

        if_data = samples[:, indices, :].astype(np.float64)

        if self.background is not None:
            if_data -= self.background[indices]

        if_data -= np.mean(if_data, axis=-1, keepdims=True)

        if self.highpass is not None:
            if_data = self.highpass.apply(if_data)

        spectra = np.fft.rfft(if_data * self.window, axis=-1)
        magnitude = np.abs(np.mean(spectra, axis=0)) * self.scale

        return 20.0 * np.log10(np.maximum(magnitude, 1e-15))


# =====================================================================
# Window helpers
# =====================================================================

def enable_dpi_awareness():
    """
    Draw sharp text and plots on scaled Windows displays (125 %, 150 %,
    ...) instead of a stretched, blurry bitmap, as IDLE does. Must run
    before the Tk window is created.
    """

    if sys.platform != "win32":
        return

    try:
        # PROCESS_SYSTEM_DPI_AWARE
        ctypes.OleDLL("shcore").SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass


def screen_work_area(window):
    """(x, y, width, height) of the screen area not covered by the taskbar."""

    if sys.platform == "win32":

        from ctypes import wintypes

        rect = wintypes.RECT()
        spi_getworkarea = 0x0030

        try:
            if ctypes.windll.user32.SystemParametersInfoW(
                spi_getworkarea, 0, ctypes.byref(rect), 0,
            ):
                return (
                    rect.left,
                    rect.top,
                    rect.right - rect.left,
                    rect.bottom - rect.top,
                )
        except (AttributeError, OSError):
            pass

    return 0, 0, window.winfo_screenwidth(), window.winfo_screenheight()


def maximize(window):

    try:
        window.state("zoomed")  # Windows, macOS
    except tk.TclError:
        try:
            window.attributes("-zoomed", True)  # X11
        except tk.TclError:
            pass


def place_window(window, size, min_size, scale):
    """
    Show the window centred on the screen at size (at least its requested
    size), or maximized when that does not fit the screen.
    """

    window.update_idletasks()

    area_x, area_y, area_width, area_height = screen_work_area(window)

    # Room for the window frame and title bar.
    frame_width = round(16 * scale)
    frame_height = round(48 * scale)

    max_width = max(1, area_width - frame_width)
    max_height = max(1, area_height - frame_height)

    preferred = (
        max(size[0], window.winfo_reqwidth()),
        max(size[1], window.winfo_reqheight()),
    )

    width = min(preferred[0], max_width)
    height = min(preferred[1], max_height)

    x = area_x + max(0, (area_width - frame_width - width) // 2)
    y = area_y + max(0, (area_height - frame_height - height) // 2)

    window.geometry(f"{width}x{height}+{x}+{y}")
    window.minsize(min(min_size[0], max_width), min(min_size[1], max_height))

    if (width, height) != preferred:
        maximize(window)

    window.deiconify()


def set_enabled(widget, enabled):
    widget.state(["!disabled"] if enabled else ["disabled"])


def wrap_to_width(event):
    """Wrap a label's text at the width the label actually has."""

    if event.width > 1:
        event.widget.configure(wraplength=event.width)


class ScrollableFrame(ttk.Frame):
    """
    Frame with a vertically scrolling body above a fixed footer.

    Put widgets into .body and .footer. When the body does not fit, it
    scrolls with the scroll bar, the mouse wheel, and keyboard focus
    (Tab); the footer always stays visible.
    """

    def __init__(self, parent, scroll_step=SCROLL_STEP, **kwargs):

        super().__init__(parent, **kwargs)

        # Missing height is taken from the body, which then scrolls; extra
        # height goes below the footer, so the footer follows the body.
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(3, weight=1000)

        self._canvas = tk.Canvas(
            self,
            borderwidth=0,
            highlightthickness=0,
            yscrollincrement=scroll_step,
        )
        self._canvas.grid(row=0, column=0, sticky="nsew")

        background = ttk.Style(self).lookup("TFrame", "background")

        if background:
            self._canvas.configure(background=background)

        self._scrollbar = ttk.Scrollbar(
            self, orient="vertical", command=self._canvas.yview,
        )
        self._scrollbar.grid(row=0, column=1, sticky="ns", padx=(4, 0))

        # The scroll bar column keeps its width while the bar is hidden,
        # so showing or hiding it never changes the layout.
        self.columnconfigure(
            1, minsize=self._scrollbar.winfo_reqwidth() + 4,
        )

        self._canvas.configure(yscrollcommand=self._on_scroll)

        self.body = ttk.Frame(self._canvas)
        self.body.columnconfigure(0, weight=1)
        self._body_item = self._canvas.create_window(
            0, 0, window=self.body, anchor="nw",
        )

        ttk.Separator(self).grid(row=1, column=0, sticky="ew", pady=(0, 8))

        self.footer = ttk.Frame(self)
        self.footer.columnconfigure(0, weight=1)
        self.footer.grid(row=2, column=0, sticky="ew")

        self._aqua = self.tk.call("tk", "windowingsystem") == "aqua"
        self._scroll_step = scroll_step
        self._wheel_steps = 0.0

        self.body.bind("<Configure>", self._on_body_configure)
        self._canvas.bind("<Configure>", self._on_canvas_configure)

        toplevel = self.winfo_toplevel()

        for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            toplevel.bind(sequence, self._on_mousewheel, add="+")

        # Generated by Tab traversal only, not by clicks or by window
        # activation, which would lose the scroll position.
        toplevel.bind("<<TraverseIn>>", self._on_traverse_in, add="+")

    def _on_body_configure(self, event):

        self._canvas.configure(scrollregion=(0, 0, event.width, event.height))
        self.fit_content()

    def fit_content(self):
        """Request the size that shows the whole body without scrolling."""

        width = self.body.winfo_reqwidth()
        height = self.body.winfo_reqheight()

        if (
            self._canvas.winfo_reqwidth() != width
            or self._canvas.winfo_reqheight() != height
        ):
            self._canvas.configure(width=width, height=height)

    def _on_canvas_configure(self, event):
        self._canvas.itemconfigure(self._body_item, width=event.width)

    def _on_scroll(self, first, last):

        self._scrollbar.set(first, last)

        if float(first) <= 0.0 and float(last) >= 1.0:
            self._scrollbar.grid_remove()
        else:
            self._scrollbar.grid()

    def _scrollable(self):

        first, last = self._canvas.yview()

        return first > 0.0 or last < 1.0

    def _on_mousewheel(self, event):

        # Path of the widget under the mouse pointer (the event goes to
        # the focus widget on some platforms).
        path = str(
            self.tk.call("winfo", "containing", event.x_root, event.y_root)
        )

        if not (path + ".").startswith(str(self) + "."):
            return

        if not self._scrollable():
            return

        if event.num == 4:
            steps = -WHEEL_STEPS
        elif event.num == 5:
            steps = WHEEL_STEPS
        elif self._aqua:
            steps = -event.delta
        else:
            # 120 per notch; touchpads send fractions of a notch.
            steps = -event.delta / 120 * WHEEL_STEPS

        self._wheel_steps += steps
        whole_steps = int(self._wheel_steps)
        self._wheel_steps -= whole_steps

        if whole_steps:
            self._canvas.yview_scroll(whole_steps, "units")

    def _on_traverse_in(self, event):
        """Scroll a widget reached with Tab into view."""

        widget = event.widget

        if (
            not isinstance(widget, tk.Misc)
            or not str(widget).startswith(str(self.body) + ".")
            or not self._scrollable()
        ):
            return

        top = widget.winfo_rooty() - self.body.winfo_rooty()
        bottom = top + widget.winfo_height()

        view_top = self._canvas.canvasy(0)
        view_height = self._canvas.winfo_height()
        body_height = max(self.body.winfo_height(), 1)

        # The canvas aligns the view to whole scroll steps.
        step = self._scroll_step

        if top < view_top:
            new_top = top // step * step
        elif bottom > view_top + view_height:
            new_top = -(-(bottom - view_height) // step) * step
        else:
            return

        self._canvas.yview_moveto(new_top / body_height)


# =====================================================================
# Real-time display window (GUI thread)
# =====================================================================

class RealTimeWindow:
    """
    Second window of the real-time mode: live range profiles of the
    ticked traces, the display processing and the recording controls.
    Talks to the radar worker through RadarGui.send_command().
    """

    def __init__(self, gui, setup):

        self.gui = gui
        self.run_id = setup.run_id
        self.config = setup.config
        self.settings = setup.settings
        self.trace_map = tuple(setup.trace_map)

        n_points = self.config.sweep_points
        self.range_axis_m = self.config.range_axis_m(n_points // 2 + 1)
        self.processor = RangeProcessor(n_points, setup.full_scale)

        self.background = None
        self.session_active = True
        self.capturing = False
        # idle, starting, recording, stopping, saving
        self.record_state = "idle"
        self.plot = None
        self._processing_parts = ()

        self.top = tk.Toplevel(gui.root)
        self.top.withdraw()
        self.top.title("X1000 Radar - Real-time display")
        self.top.protocol("WM_DELETE_WINDOW", self._on_close)

        self._create_variables()
        self._build()
        self._update_controls()
        self._update_processing_parts()
        self._init_plot()

        self.top.update_idletasks()
        self._controls.fit_content()
        self.top.update_idletasks()

        place_window(
            self.top,
            gui.px_size(DISPLAY_WINDOW_SIZE),
            (
                self._controls.winfo_reqwidth() + gui.px(MIN_PLOT_WIDTH),
                gui.px(MIN_WINDOW_HEIGHT),
            ),
            gui.scale,
        )

    def destroy(self):

        self.plot = None
        self.top.destroy()

    def _on_close(self):
        self.gui.display_window_closed(self)

    # -----------------------------------------------------------------
    # Layout
    # -----------------------------------------------------------------

    def _create_variables(self):

        self.values = {
            key: tk.StringVar(value=value)
            for key, value in DISPLAY_DEFAULT_VALUES.items()
        }

        self.trace_visible = {
            trace: tk.BooleanVar(value=True)
            for trace in self.trace_map
        }

        self.subtract_background = tk.BooleanVar(value=False)
        self.highpass = tk.BooleanVar(value=False)

        self.background_text = tk.StringVar(
            value="No background captured."
        )
        self.recording_text = tk.StringVar(value=self._idle_recording_text())
        self.status_text = tk.StringVar(value="Waiting for the first frame...")

    def _idle_recording_text(self):

        settings = self.settings

        if settings.frames_to_record:
            length = f"{settings.frames_to_record} frames"
        else:
            length = "until Stop recording"

        return (
            f"Not recording. Record saves the raw frames ({length}) as "
            f".{settings.file_format} in {settings.output_dir}."
        )

    def _entry(self, parent, key, width):

        return ttk.Entry(
            parent,
            textvariable=self.values[key],
            width=width,
            justify="right",
        )

    def _build(self):

        gui = self.gui

        self.top.columnconfigure(0, weight=1)
        self.top.rowconfigure(0, weight=1)

        plot_frame = ttk.Frame(self.top, padding=(8, 8, 4, 8))
        plot_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)
        self._plot_frame = plot_frame

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

        self._status_label = ttk.Label(
            plot_frame,
            textvariable=self.status_text,
            anchor="w",
            justify="left",
            padding=(0, 4),
            wraplength=gui.px(600),
        )
        self._status_label.grid(row=2, column=0, sticky="ew")
        self._status_label.bind("<Configure>", self._on_status_configure)
        self._status_width = 0
        self._status_height = 0

        self._controls = ScrollableFrame(
            self.top,
            scroll_step=gui.px(SCROLL_STEP),
            padding=(4, 8, 8, 8),
        )
        self._controls.grid(row=0, column=1, sticky="ns")

        self._build_trace_frame(self._controls.body)
        self._build_processing_frame(self._controls.body)
        self._build_plot_frame(self._controls.body)
        self._build_recording_frame(self._controls.footer)

    def _build_trace_frame(self, parent):

        frame = ttk.LabelFrame(parent, text="Traces shown", padding=8)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        for column, trace in enumerate(self.trace_map):

            ttk.Checkbutton(
                frame,
                text=TRACE_LABELS.get(trace, str(trace)),
                variable=self.trace_visible[trace],
                command=self._init_plot,
            ).grid(row=0, column=column, sticky="w", padx=(0, 8))

    def _build_processing_frame(self, parent):

        gui = self.gui

        frame = ttk.LabelFrame(
            parent, text="Processing (display only)", padding=8,
        )
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        self._subtract_check = ttk.Checkbutton(
            frame,
            text="Subtract the background",
            variable=self.subtract_background,
            command=self._apply_processing,
        )
        self._subtract_check.grid(row=0, column=0, columnspan=3, sticky="w")

        self._capture_button = ttk.Button(
            frame,
            text="Capture background",
            command=self._on_capture_background,
        )
        self._capture_button.grid(row=1, column=0, sticky="w", pady=(4, 2))

        self._background_entry = self._entry(frame, "background_frames", 6)
        self._background_entry.grid(row=1, column=1, sticky="e", pady=(4, 2))

        ttk.Label(frame, text="frames").grid(
            row=1, column=2, sticky="w", padx=(4, 0), pady=(4, 2),
        )

        ttk.Label(
            frame,
            textvariable=self.background_text,
            justify="left",
            wraplength=gui.px(CONTROL_TEXT_WIDTH),
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(0, 4))

        ttk.Separator(frame).grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=6,
        )

        ttk.Checkbutton(
            frame,
            text="High-pass filter (antenna coupling)",
            variable=self.highpass,
            command=self._apply_processing,
        ).grid(row=4, column=0, columnspan=3, sticky="w")

        rows = (
            ("highpass_cutoff_m", "Cutoff range", "m"),
            ("highpass_order", "Butterworth order", ""),
        )

        for row, (key, label, unit) in enumerate(rows, start=5):

            ttk.Label(frame, text=label).grid(
                row=row, column=0, sticky="w", pady=2,
            )

            entry = self._entry(frame, key, 6)
            entry.grid(row=row, column=1, sticky="e", pady=2)
            entry.bind("<Return>", self._apply_processing)

            ttk.Label(frame, text=unit).grid(
                row=row, column=2, sticky="w", padx=(4, 0), pady=2,
            )

        ttk.Button(
            frame, text="Apply", command=self._apply_processing,
        ).grid(row=7, column=0, columnspan=3, sticky="e", pady=(4, 0))

    def _build_plot_frame(self, parent):

        frame = ttk.LabelFrame(parent, text="Plot", padding=8)
        frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))

        ttk.Label(frame, text="from").grid(row=0, column=1)
        ttk.Label(frame, text="to").grid(row=0, column=2)

        rows = (
            ("Range [m]", "plot_min_range_m", "plot_max_range_m"),
            ("Level [dBFS]", "plot_min_db", "plot_max_db"),
            ("Peak search [m]", "peak_min_range_m", "peak_max_range_m"),
        )

        for row, (label, low_key, high_key) in enumerate(rows, start=1):

            ttk.Label(frame, text=label).grid(
                row=row, column=0, sticky="w", padx=(0, 6), pady=2,
            )

            for column, key in ((1, low_key), (2, high_key)):

                entry = self._entry(frame, key, 7)
                entry.grid(row=row, column=column, padx=2, pady=2)
                entry.bind("<Return>", self._init_plot)

        ttk.Button(frame, text="Apply", command=self._init_plot).grid(
            row=4, column=0, columnspan=3, sticky="e", pady=(4, 0),
        )

    def _build_recording_frame(self, parent):

        frame = ttk.LabelFrame(parent, text="Recording (raw data)", padding=8)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        self._record_button = ttk.Button(
            frame, text="Record", command=self._on_record,
        )
        self._record_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self._stop_record_button = ttk.Button(
            frame, text="Stop recording", command=self._on_stop_recording,
        )
        self._stop_record_button.grid(
            row=0, column=1, sticky="ew", padx=(4, 0),
        )

        ttk.Label(
            frame,
            textvariable=self.recording_text,
            justify="left",
            wraplength=self.gui.px(CONTROL_TEXT_WIDTH),
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def _update_controls(self):

        active = self.session_active

        set_enabled(
            self._record_button, active and self.record_state == "idle",
        )
        set_enabled(
            self._stop_record_button,
            active and self.record_state == "recording",
        )
        set_enabled(self._capture_button, active and not self.capturing)
        set_enabled(self._background_entry, active and not self.capturing)
        set_enabled(self._subtract_check, self.background is not None)

    # -----------------------------------------------------------------
    # Processing and background
    # -----------------------------------------------------------------

    def _apply_processing(self, event=None):

        processor = self.processor

        processor.background = None

        if self.subtract_background.get() and self.background is not None:
            processor.background = self.background.samples

        processor.highpass = None

        if self.highpass.get():

            try:
                processor.highpass = design_highpass(
                    parse_number(
                        self.values["highpass_cutoff_m"].get(),
                        "High-pass cutoff range",
                    ),
                    parse_integer(
                        self.values["highpass_order"].get(),
                        "Butterworth order",
                        minimum=1,
                    ),
                    self.config,
                )

            except ValueError as error:
                self.highpass.set(False)
                messagebox.showerror(
                    "High-pass filter",
                    f"{error}\n\nThe filter is off.",
                    parent=self.top,
                )

        self._update_processing_parts()
        self.gui.log(
            "Display: " + ", ".join(self._processing_parts).lower() + "."
        )

    def _update_processing_parts(self):

        background = "ON" if self.processor.background is not None else "OFF"
        highpass = self.processor.highpass

        if highpass is not None:
            highpass_text = f"{highpass.cutoff_m:g} m, order {highpass.order}"
        else:
            highpass_text = "OFF"

        self._processing_parts = (
            f"Background subtraction {background}",
            f"High-pass {highpass_text}",
        )

    def _on_capture_background(self):

        try:
            n_frames = parse_integer(
                self.values["background_frames"].get(),
                "Background frames",
                minimum=1,
            )
        except ValueError as error:
            messagebox.showerror("Background", str(error), parent=self.top)
            return

        if not messagebox.askokcancel(
            "Empty-scene background",
            "Remove the corner reflector and all other targets from the "
            "radar field of view. Do not move the radar, its mounting or "
            "the surrounding equipment.\n\n"
            f"The next {n_frames} frames are averaged into the "
            f"background.\n\n"
            "Click OK when the scene is empty and stable.",
            parent=self.top,
        ):
            return

        # The radar may have stopped while the question was open.
        if not self.session_active:
            return

        self.capturing = True
        self.background_text.set(
            f"Capturing the background: 0 of {n_frames} frames..."
        )
        self._update_controls()

        self.gui.send_command("background", n_frames)

    def background_progress(self, done, total):

        if self.capturing:
            self.background_text.set(
                f"Capturing the background: {done} of {total} frames..."
            )

    def set_background(self, background):

        self.background = background
        self.capturing = False
        self.subtract_background.set(True)

        stamp = datetime.fromtimestamp(background.timestamp_s)

        self.background_text.set(
            f"Background of {background.n_frames} frames captured at "
            f"{stamp:%H:%M:%S}; its subtraction is on. Place the targets "
            f"now. Recordings started from now on also save it."
        )

        self._update_controls()
        self._apply_processing()

    # -----------------------------------------------------------------
    # Recording
    # -----------------------------------------------------------------

    def _on_record(self):

        self.record_state = "starting"
        self.recording_text.set("Starting the recording...")
        self._update_controls()

        self.gui.send_command("record", True)

    def _on_stop_recording(self):

        self.record_state = "stopping"
        self.recording_text.set("Stopping the recording...")
        self._update_controls()

        self.gui.send_command("record", False)

    def recording_started(self, path):

        self.record_state = "recording"
        self.recording_text.set(f"Recording to {Path(path).name}...")
        self._update_controls()

    def recording_status(self, status):

        if self.record_state in ("recording", "stopping"):
            self.recording_text.set(recording_status_text(status))

    def recording_saving(self, n_frames, n_bytes):

        self.record_state = "saving"
        self.recording_text.set(
            f"Saving {n_frames} frames ({format_bytes(n_bytes)} of raw "
            f"data)..."
        )
        self._update_controls()

    def recording_ended(self, path, n_frames, duration_s, file_bytes):

        self.record_state = "idle"

        if path is not None:
            self.recording_text.set(
                f"Saved {n_frames} frames ({format_duration(duration_s)}) "
                f"to {Path(path).name}: {format_bytes(file_bytes)}."
            )
        elif n_frames:
            self.recording_text.set(
                "Saving failed; the raw frames are kept in a .part file "
                "(see the log)."
            )
        else:
            self.recording_text.set("Not recording. Nothing was saved.")

        self._update_controls()

    def session_ended(self):

        self.session_active = False

        if self.capturing:
            self.capturing = False
            self.background_text.set("Background capture stopped.")

        if self.record_state != "idle":
            self.record_state = "idle"
            self.recording_text.set("Not recording.")

        self.top.title("X1000 Radar - Real-time display (stopped)")
        self._update_controls()

    # -----------------------------------------------------------------
    # Plotting
    # -----------------------------------------------------------------

    def _show_message(self, message):

        self.plot = None

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
            wrap=True,
        )

        self.canvas.draw_idle()

    def _init_plot(self, event=None):

        try:
            x_min, x_max = parse_interval(
                self.values["plot_min_range_m"].get(),
                self.values["plot_max_range_m"].get(),
                "Plot range",
            )
            y_min, y_max = parse_interval(
                self.values["plot_min_db"].get(),
                self.values["plot_max_db"].get(),
                "Plot level",
            )
            peak_min, peak_max = parse_interval(
                self.values["peak_min_range_m"].get(),
                self.values["peak_max_range_m"].get(),
                "Peak search",
            )
        except ValueError as error:
            messagebox.showerror("Plot", str(error), parent=self.top)
            return

        visible = [
            trace for trace in self.trace_map
            if self.trace_visible[trace].get()
        ]

        if not visible:
            self._show_message("Tick at least one trace to show it.")
            return

        range_axis_m = self.range_axis_m
        last_range_m = float(range_axis_m[-1])
        x_max = min(x_max, last_range_m)

        if x_max <= x_min:
            x_min, x_max = 0.0, last_range_m
            self.gui.log(
                "The plot range lies outside the measured range "
                f"(0 - {last_range_m:.3f} m); the full range is shown."
            )

        peak_indices = np.flatnonzero(
            (range_axis_m >= peak_min) & (range_axis_m <= peak_max)
        )

        if peak_indices.size == 0:
            self.gui.log(
                "No range bins lie in the peak-search interval; "
                "peak marking is off."
            )

        self.figure.clear()

        axes = self.figure.subplots(
            nrows=len(visible),
            ncols=1,
            sharex=True,
            squeeze=False,
        )[:, 0]

        empty = np.full(range_axis_m.shape, np.nan)

        artists = {}
        titles = []

        for axis, trace in zip(axes, visible):

            name = f"Trace {TRACE_LABELS.get(trace, trace)}"

            line, = axis.plot(
                range_axis_m,
                empty,
                linewidth=1.0,
                label=name,
                color=f"C{trace}",
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

            titles.append(axis.set_title(name, wrap=True))
            axis.set_xlim(x_min, x_max)
            axis.set_ylim(y_min, y_max)
            axis.grid(True)

            # Animated so it is blitted on top of the curve.
            legend = axis.legend(loc="lower right")
            legend.set_animated(True)

            artists[trace] = TraceArtists(line, marker, label, legend)

        axes[-1].set_xlabel("One-way range (m)")

        # One label for all rows: a label per row is taller than the
        # rows of a small plot, and the layout then collapses.
        self.figure.supylabel("Magnitude (dBFS)")

        titles.append(
            self.figure.suptitle("Real-time range profiles", wrap=True)
        )

        self.plot = {
            "traces": visible,
            "indices": [self.trace_map.index(trace) for trace in visible],
            "peak_indices": peak_indices,
            "artists": artists,
            "titles": titles,
            "compact": None,
            "background": None,
        }

        self._fit_plot_to_size()
        self.canvas.draw_idle()

    def _fit_plot_to_size(self):
        """
        Compact plot when the rows are short (small screen or window):
        no titles, and one line per row with the curve name and its peak
        instead of the legend and the three-line peak label.
        """

        plot = self.plot

        if plot is None:
            return

        row_height_pt = (
            72.0 * self.figure.get_figheight() / len(plot["artists"])
        )
        compact = row_height_pt < PLOT_COMPACT_ROW_HEIGHT_PT

        if compact == plot["compact"]:
            return

        plot["compact"] = compact

        for title in plot["titles"]:
            title.set_visible(not compact)

        for trace_artists in plot["artists"].values():
            trace_artists.legend.set_visible(not compact)
            trace_artists.label.set_fontsize("small" if compact else "medium")
            trace_artists.label.set_text(self._peak_text(trace_artists))

    def _peak_text(self, trace_artists):

        name = trace_artists.line.get_label()
        peak = trace_artists.peak

        if self.plot["compact"]:

            if peak is None:
                return name

            return f"{name}: peak {peak[0]:.4f} m, {peak[1]:.1f} dBFS"

        if peak is None:
            return ""

        return (
            f"Strongest target peak\n"
            f"Range: {peak[0]:.4f} m\n"
            f"Level: {peak[1]:.1f} dBFS"
        )

    def _dynamic_artists(self):

        if self.plot is None:
            return []

        return [
            artist
            for trace_artists in self.plot["artists"].values()
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
            self.plot is None
            or event.canvas is not self.canvas
            or self.canvas.is_saving()
        ):
            return

        self.plot["background"] = self.canvas.copy_from_bbox(
            self.figure.bbox
        )

        # FigureCanvasTkAgg.draw() blits the whole buffer afterwards.
        self._draw_dynamic_artists()

    def _on_canvas_resize(self, event):

        if self.plot is not None:
            self.plot["background"] = None
            self._fit_plot_to_size()

    def draw(self, item):
        """Show one frame (DisplayFrame)."""

        plot = self.plot

        if plot is None or item.run_id != self.run_id:
            return

        profiles = self.processor.profiles_db(
            item.frame.samples, plot["indices"],
        )

        range_axis_m = self.range_axis_m
        peak_indices = plot["peak_indices"]

        peak_messages = []

        for trace, profile in zip(plot["traces"], profiles):

            artists = plot["artists"][trace]
            artists.line.set_ydata(profile)

            if peak_indices.size == 0:
                continue

            peak_index = int(peak_indices[np.argmax(profile[peak_indices])])
            peak_range_m = float(range_axis_m[peak_index])
            peak_level_db = float(profile[peak_index])

            artists.marker.set_data([peak_range_m], [peak_level_db])
            artists.peak = (peak_range_m, peak_level_db)
            artists.label.set_text(self._peak_text(artists))

            peak_messages.append(f"T{trace}: {peak_range_m:.3f} m")

        self._set_status([
            f"Frame {item.frame_number}",
            f"{item.rate_hz:.1f} frames/s",
            f"acquisition {item.acquisition_ms:.1f} ms",
            *self._processing_parts,
            *peak_messages,
        ])

        if plot["background"] is None:
            # No cached background yet (first frame or resize).
            self.canvas.draw_idle()
            return

        self.canvas.restore_region(plot["background"])
        self._draw_dynamic_artists()

        for trace_artists in plot["artists"].values():
            self.canvas.blit(trace_artists.line.axes.bbox)

    def _on_status_configure(self, event):

        if event.width > 1 and event.width != self._status_width:
            self._status_width = event.width
            self._status_label.configure(wraplength=event.width)
            self._status_height = 0
            self._plot_frame.rowconfigure(2, minsize=0)

    def _set_status(self, parts):
        """Show the parts joined by " | ", wrapped between the parts only."""

        self.status_text.set(
            " | ".join(
                part.replace(" ", "\N{NO-BREAK SPACE}") for part in parts
            )
        )

        # The status line may grow but not shrink while the width stays
        # the same: when its line count changes with the digits of the
        # numbers, the plot would otherwise be resized and fully redrawn.
        height = self._status_label.winfo_reqheight()

        if height > self._status_height:
            self._status_height = height
            self._plot_frame.rowconfigure(2, minsize=height)


# =====================================================================
# Main window
# =====================================================================

class RadarGui:

    def __init__(self, root):

        self.root = root
        self.root.title("X1000 Radar")

        # Hidden until it has its final size, see place_window.
        self.root.withdraw()

        # Pixels per pixel of a 100 % display, e.g. 1.5 at 150 %.
        self.scale = max(1.0, self.root.winfo_fpixels("1i") / 96.0)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._events = queue.Queue()
        self._commands = queue.Queue()
        self._stop_event = threading.Event()

        self._display_lock = threading.Lock()
        self._pending_display = None

        self._worker = None
        self._running = False
        self._close_requested = False
        self._alive = True

        self._run_id = 0
        self._last_saved_path = None
        self._display = None
        self._progress_busy_on = False

        self._create_variables()
        self._build_layout()
        self._update_widget_states()

        self.root.update_idletasks()
        self._controls.fit_content()
        self.root.update_idletasks()

        place_window(
            self.root,
            self.px_size(MAIN_WINDOW_SIZE),
            (
                self._controls.winfo_reqwidth() + self.px(MIN_LOG_WIDTH),
                self.px(MIN_WINDOW_HEIGHT),
            ),
            self.scale,
        )

        self.log(
            "Ready. Set the radar parameters and the recording options, "
            "choose the mode and press Start."
        )

        self.root.after(POLL_INTERVAL_MS, self._poll_events)

    def px(self, pixels):
        """Pixels of a 100 % display -> pixels of this display."""
        return round(pixels * self.scale)

    def px_size(self, size):
        return self.px(size[0]), self.px(size[1])

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

        self.mode = tk.StringVar(value="display")
        self.output_dir = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.file_format = tk.StringVar(value="mat")
        self.record_length = tk.StringVar(value="until_stop")
        self.capture_background = tk.BooleanVar(value=False)

        self.status_text = tk.StringVar(value="Idle.")

    def _build_layout(self):

        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # The options scroll when the window is too short for them;
        # Start/Stop stay visible below.
        self._controls = ScrollableFrame(
            self.root,
            scroll_step=self.px(SCROLL_STEP),
            padding=(8, 8, 4, 8),
        )
        self._controls.grid(row=0, column=0, sticky="ns")

        self._input_widgets = []

        self._build_parameter_frame(self._controls.body)
        self._build_mode_frame(self._controls.body)
        self._build_recording_frame(self._controls.body)
        self._build_run_frame(self._controls.footer)

        log_frame = ttk.Frame(self.root, padding=(4, 8, 8, 8))
        log_frame.grid(row=0, column=1, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=1)

        ttk.Label(log_frame, text="Log").grid(row=0, column=0, sticky="w")

        self.log_text = ScrolledText(
            log_frame, height=10, width=60, state="disabled", wrap="word",
        )
        self.log_text.grid(row=1, column=0, sticky="nsew")

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

    def _build_mode_frame(self, parent):

        frame = ttk.LabelFrame(parent, text="2. Mode", padding=8)
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))

        modes = (
            ("display", "Real-time display (Record in the display window)"),
            ("save", "Save data only (progress bar, no display)"),
        )

        for row, (value, text) in enumerate(modes):

            radio = ttk.Radiobutton(
                frame,
                text=text,
                value=value,
                variable=self.mode,
                command=self._update_widget_states,
            )
            radio.grid(row=row, column=0, sticky="w")

            self._input_widgets.append(radio)

    def _build_recording_frame(self, parent):

        frame = ttk.LabelFrame(
            parent, text="3. Recording (raw data)", padding=8,
        )
        frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Save folder").grid(
            row=0, column=0, columnspan=3, sticky="w",
        )

        folder_entry = ttk.Entry(frame, textvariable=self.output_dir, width=30)
        folder_entry.grid(row=1, column=0, columnspan=2, sticky="ew", pady=2)

        browse = ttk.Button(
            frame, text="Browse...", command=self._browse_output_dir,
        )
        browse.grid(row=1, column=2, sticky="w", padx=(4, 0), pady=2)

        self._input_widgets += [folder_entry, browse]

        ttk.Label(frame, text="File format").grid(
            row=2, column=0, sticky="w", pady=2,
        )

        formats = ttk.Frame(frame)
        formats.grid(row=2, column=1, columnspan=2, sticky="w", pady=2)

        for column, (value, text) in enumerate(
            (("mat", "MATLAB (.mat)"), ("npz", "NumPy (.npz)"))
        ):

            radio = ttk.Radiobutton(
                formats, text=text, value=value, variable=self.file_format,
            )
            radio.grid(row=0, column=column, sticky="w", padx=(0, 8))

            self._input_widgets.append(radio)

        ttk.Label(frame, text="Length").grid(
            row=3, column=0, sticky="nw", pady=2,
        )

        lengths = ttk.Frame(frame)
        lengths.grid(row=3, column=1, columnspan=2, sticky="w", pady=2)

        until_stop = ttk.Radiobutton(
            lengths,
            text="until Stop",
            value="until_stop",
            variable=self.record_length,
            command=self._update_widget_states,
        )
        until_stop.grid(row=0, column=0, columnspan=3, sticky="w")

        fixed = ttk.Radiobutton(
            lengths,
            value="frames",
            variable=self.record_length,
            command=self._update_widget_states,
        )
        fixed.grid(row=1, column=0, sticky="w")

        self._frames_entry = ttk.Entry(
            lengths,
            textvariable=self.values["frames_to_record"],
            width=8,
            justify="right",
        )
        self._frames_entry.grid(row=1, column=1, sticky="w")

        ttk.Label(lengths, text="frames").grid(
            row=1, column=2, sticky="w", padx=(4, 0),
        )

        self._input_widgets += [until_stop, fixed]

        background = ttk.Frame(frame)
        background.grid(row=4, column=0, columnspan=3, sticky="w", pady=(6, 0))

        self._background_check = ttk.Checkbutton(
            background,
            text="Capture an empty-scene background of",
            variable=self.capture_background,
            command=self._update_widget_states,
        )
        self._background_check.grid(row=0, column=0, sticky="w")

        self._background_entry = ttk.Entry(
            background,
            textvariable=self.values["background_frames"],
            width=6,
            justify="right",
        )
        self._background_entry.grid(row=0, column=1, padx=(4, 0))

        ttk.Label(background, text="frames first").grid(
            row=0, column=2, sticky="w", padx=(4, 0),
        )

        ttk.Label(
            frame,
            text="(save mode; the display window has its own button)",
            foreground="gray",
        ).grid(row=5, column=0, columnspan=3, sticky="w", padx=(22, 0))

    def _build_run_frame(self, parent):

        frame = ttk.Frame(parent)
        frame.grid(row=0, column=0, sticky="ew")
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

        status = ttk.Label(
            frame,
            textvariable=self.status_text,
            wraplength=self.px(300),
            justify="left",
        )
        status.grid(row=2, column=0, columnspan=2, sticky="ew")
        status.bind("<Configure>", wrap_to_width)

    # -----------------------------------------------------------------
    # Widget helpers
    # -----------------------------------------------------------------

    def _update_widget_states(self):

        running = self._running
        save_mode = self.mode.get() == "save"

        for widget in self._input_widgets:
            set_enabled(widget, not running)

        set_enabled(
            self._frames_entry,
            not running and self.record_length.get() == "frames",
        )
        set_enabled(self._background_check, not running and save_mode)
        set_enabled(
            self._background_entry,
            not running and save_mode and self.capture_background.get(),
        )

        set_enabled(self.start_button, not running)
        set_enabled(
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

    def _progress_value(self, fraction):

        if self._progress_busy_on:
            self.progress.stop()
            self._progress_busy_on = False

        self.progress.configure(mode="determinate", maximum=1.0, value=fraction)

    def _progress_busy(self):

        if not self._progress_busy_on:
            self.progress.configure(mode="indeterminate", value=0.0)
            self.progress.start(15)
            self._progress_busy_on = True

    def log(self, message):

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

    def _read_settings(self):
        """Parse the inputs; the radar checks the values itself."""

        def value(key):
            return self.values[key].get()

        traces = tuple(
            trace for trace in (0, 1, 2) if self.trace_enabled[trace].get()
        )

        if not traces:
            raise ValueError("Select at least one trace.")

        folder = self.output_dir.get().strip()

        if not folder:
            raise ValueError("Choose a save folder.")

        output_dir = Path(folder).expanduser().resolve()

        display = self.mode.get() == "display"

        if not display:

            # Fail now rather than after the radar setup and background.
            try:
                output_dir.mkdir(parents=True, exist_ok=True)

                with tempfile.TemporaryFile(dir=output_dir):
                    pass

            except OSError as error:
                raise ValueError(
                    f"The save folder cannot be used:\n{output_dir}\n\n{error}"
                ) from None

        frames_to_record = 0

        if self.record_length.get() == "frames":
            frames_to_record = parse_integer(
                value("frames_to_record"), "Frames to record", minimum=1,
            )

        background_frames = 0

        if not display and self.capture_background.get():
            background_frames = parse_integer(
                value("background_frames"), "Background frames", minimum=1,
            )

        return RunSettings(
            f_start_hz=parse_number(value("f_start_ghz"), "Start frequency") * 1e9,
            f_stop_hz=parse_number(value("f_stop_ghz"), "Stop frequency") * 1e9,
            sweep_time_s=parse_number(value("sweep_time_ms"), "Sweep time") / 1e3,
            sweep_period_s=parse_number(value("sweep_period_ms"), "Sweep period") / 1e3,
            sweep_count=parse_integer(value("sweep_count"), "Sweeps per frame"),
            traces=traces,
            display=display,
            output_dir=output_dir,
            file_format=self.file_format.get(),
            frames_to_record=frames_to_record,
            background_frames=background_frames,
        )

    def _log_settings(self, settings):

        if settings.frames_to_record:
            length = f"{settings.frames_to_record} frames"
        else:
            length = "until Stop"

        lines = [
            "Starting:",
            f"  Frequency    : {settings.f_start_hz / 1e9:g} - "
            f"{settings.f_stop_hz / 1e9:g} GHz",
            f"  Sweep        : time {settings.sweep_time_s * 1e3:g} ms, "
            f"period {settings.sweep_period_s * 1e3:g} ms, "
            f"{settings.sweep_count} sweep(s) per frame",
            f"  Traces       : {list(settings.traces)}",
            "  Mode         : "
            + ("real-time display" if settings.display else "save data"),
            f"  Recording    : {length}, .{settings.file_format} in "
            f"{settings.output_dir}",
        ]

        if not settings.display:

            if settings.background_frames:
                background = f"{settings.background_frames} frames first"
            else:
                background = "none"

            lines.append(f"  Background   : {background}")

        self.log("\n".join(lines))

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

        if self._display is not None:
            self._display.destroy()
            self._display = None

        self._run_id += 1
        self._last_saved_path = None

        self._stop_event.clear()
        self._take_commands()

        with self._display_lock:
            self._pending_display = None

        self._running = True
        self._update_widget_states()

        self._progress_value(0.0)
        self._set_status("Connecting to the radar...")
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
            "The radar is running.\n\n"
            "Stop it and close the window? A running recording is saved "
            "first.",
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

    def display_window_closed(self, window):
        """Closing the display window stops the radar."""

        if window is self._display:

            self._display = None

            if self._running:
                self._on_stop()

        window.destroy()

    # -----------------------------------------------------------------
    # Communication with the worker thread
    # -----------------------------------------------------------------

    def _post(self, kind, *payload):
        self._events.put((kind, *payload))

    def send_command(self, command, value):
        """Ask the worker to act between two frames (GUI thread)."""

        self._commands.put((command, value))

    def _take_commands(self):

        commands = []

        while True:
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                return commands

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

            if done >= total or now - last_post[0] >= STATUS_INTERVAL_S:
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

            if item is not None and self._display is not None:
                self._display.draw(item)

        except Exception:
            self.log("GUI error:\n" + traceback.format_exc())

        finally:
            if self._alive:
                self.root.after(POLL_INTERVAL_MS, self._poll_events)

    def _handle_event(self, event):

        kind, *payload = event

        handler = {
            "log": self.log,
            "status": self._set_status,
            "progress": self._on_progress,
            "config": self._on_config,
            "ask": self._answer_question,
            "display_init": self._on_display_init,
            "background_progress": self._on_background_progress,
            "background": self._on_background,
            "recording_started": self._on_recording_started,
            "recording": self._on_recording_status,
            "recording_saving": self._on_recording_saving,
            "recording_ended": self._on_recording_ended,
            "error": self._on_error,
            "finished": self._on_worker_finished,
        }[kind]

        handler(*payload)

    def _answer_question(self, title, message, reply):

        if self._stop_event.is_set() or self._close_requested:
            reply.put(False)
            return

        answer = messagebox.askokcancel(title, message, parent=self.root)
        reply.put(bool(answer))

    def _on_progress(self, done, total, label):

        self._progress_value(done / max(total, 1))
        self._set_status(f"{label}: {done}/{total}")

    def _on_config(self, config):

        self.log(
            "Radar configured:\n"
            f"  Frequency    : {config.frequency_start_hz / 1e9:.6f} - "
            f"{config.frequency_stop_hz / 1e9:.6f} GHz (bandwidth "
            f"{config.bandwidth_hz / 1e9:.6f} GHz, centre "
            f"{config.center_frequency_hz / 1e9:.6f} GHz)\n"
            f"  Sweep        : time {config.sweep_time_s * 1e3:.6g} ms, "
            f"period {config.sweep_period_s * 1e3:.6g} ms, "
            f"{config.sweep_count} sweep(s) per frame\n"
            f"  Samples      : {config.sweep_points} per sweep, interval "
            f"{config.sample_interval_s * 1e9:.3f} ns\n"
            f"  Range        : resolution "
            f"{config.range_resolution_m * 1e3:.3f} mm, maximum "
            f"{config.max_range_m:.3f} m\n"
            f"  Traces       : {list(config.traces)}"
        )

        self._set_status("Radar configured.")

    def _on_display_init(self, setup):

        if setup.run_id != self._run_id:
            return

        self._display = RealTimeWindow(self, setup)

        self._set_status(
            "Real-time display running. Record in the display window "
            "saves the raw data."
        )

    def _on_background_progress(self, done, total):

        if self._display is not None:
            self._display.background_progress(done, total)

    def _on_background(self, background):

        self.log(f"Background of {background.n_frames} frames captured.")

        if self._display is not None:
            self._display.set_background(background)

    def _on_recording_started(self, path, frames_limit):

        if frames_limit:
            self._progress_value(0.0)
        else:
            self._progress_busy()

        self._set_status(f"Recording to {Path(path).name}...")

        if self._display is not None:
            self._display.recording_started(path)

    def _on_recording_status(self, status):

        if status.frames_limit:
            self._progress_value(status.n_frames / status.frames_limit)

        self._set_status(recording_status_text(status))

        if self._display is not None:
            self._display.recording_status(status)

    def _on_recording_saving(self, n_frames, n_bytes):

        self._progress_busy()
        self._set_status(
            f"Saving {n_frames} frames ({format_bytes(n_bytes)} of raw "
            f"data)..."
        )

        if self._display is not None:
            self._display.recording_saving(n_frames, n_bytes)

    def _on_recording_ended(self, path, n_frames, duration_s, file_bytes):

        self._progress_value(1.0 if path is not None else 0.0)

        if path is not None:

            self._last_saved_path = path

            text = (
                f"Saved {n_frames} frames ({format_duration(duration_s)}) "
                f"to {path}: {format_bytes(file_bytes)}."
            )

            self.log(text)
            self._set_status(text)

        elif n_frames:
            self._set_status("Saving the recording failed (see the log).")

        if self._display is not None:
            self._display.recording_ended(path, n_frames, duration_s, file_bytes)

    def _on_error(self, message, details):

        self.log(f"ERROR: {message}\n{details}")

        if not self._close_requested:
            messagebox.showerror("Radar error", message, parent=self.root)

    def _on_worker_finished(self, run_id, outcome):

        if run_id != self._run_id:
            return

        self._running = False
        self._worker = None
        self._update_widget_states()

        if self._progress_busy_on:
            self._progress_value(0.0)

        status = {
            "completed": "Finished.",
            "stopped": "Stopped.",
            "cancelled": "Cancelled.",
            "failed": "Stopped because of an error (see the log).",
        }[outcome]

        if self._last_saved_path is not None:
            status += f"\nLast recording: {self._last_saved_path}"

        self._set_status(status)

        if self._display is not None:
            self._display.session_ended()

        if self._close_requested:
            self._destroy()

    # -----------------------------------------------------------------
    # Worker thread (radar access only happens here)
    # -----------------------------------------------------------------

    def _run_session(self, settings, run_id):

        device = None
        session = None
        outcome = "completed"

        try:

            self._post("status", "Searching for the radar...")

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
                TRACES=settings.traces,
            )

            self._post("config", config)

            session = Session(
                device=device,
                config=config,
                settings=settings,
                run_id=run_id,
                device_id=",".join(device.core.idn()),
            )

            if self._stop_event.is_set():
                raise AcquisitionCancelled(
                    "Stopped before the acquisition started."
                )

            if settings.display:
                self._run_display(session)
            else:
                self._run_save(session)

            if self._stop_event.is_set():
                outcome = "stopped"

        except AcquisitionCancelled as error:

            outcome = "stopped" if self._stop_event.is_set() else "cancelled"

            self._post("log", str(error))

        except Exception as error:

            outcome = "failed"

            self._post(
                "error",
                f"{type(error).__name__}: {error}",
                traceback.format_exc(),
            )

        finally:

            if session is not None and session.invalid_frames:
                self._post(
                    "log",
                    f"The radar marked {session.invalid_frames} frames as "
                    f"invalid.",
                )

            if device is not None:

                try:
                    device.close()
                    self._post("log", "Radar connection closed.")

                except Exception as error:
                    self._post(
                        "log", f"Closing the radar connection failed: {error}",
                    )

            if session is not None:
                for saver in session.savers:
                    saver.join()

            self._post("finished", run_id, outcome)

    def _acquire(self, session):

        frame = acquire_frame(session.device)

        if frame.invalid:

            session.invalid_frames += 1

            if session.invalid_frames == 1:
                self._post(
                    "log",
                    "Warning: the radar marked a frame as invalid; it is "
                    "kept. If this repeats, check radar_health.json.",
                )

        return frame

    def _capture_background(self, session):
        """Save mode: empty-scene background with instructions."""

        n_frames = session.settings.background_frames

        if not self._ask(
            "Empty-scene background",
            "Remove the corner reflector and all other targets from the "
            "radar field of view.\n\n"
            "Do not move the radar, its mounting or the surrounding "
            "equipment.\n\n"
            f"The radar averages the next {n_frames} frames.\n\n"
            "Click OK when the scene is empty and stable.",
        ):
            raise AcquisitionCancelled("Background capture cancelled.")

        self._post("status", "Capturing the empty-scene background...")

        started = time.perf_counter()

        background = capture_background(
            session.device,
            n_frames,
            progress_callback=self._make_progress_callback("Background"),
            stop_event=self._stop_event,
        )

        self._post(
            "log",
            f"Background captured from {n_frames} frames in "
            f"{time.perf_counter() - started:.2f} s.",
        )

        if not self._ask(
            "Background captured",
            "Place the targets in the scene now.\n\n"
            "Do not move the radar or the surrounding equipment.\n\n"
            "Click OK to start the recording.",
        ):
            raise AcquisitionCancelled(
                "Recording cancelled after the background capture."
            )

        return background

    def _start_recording(self, session, background):
        """
        Start a recording with the next frame. The radar clock is set to the
        PC clock first, so that the sweep timestamps share the time base of
        the stage position log (actuator_logger.m).
        """

        settings = session.settings
        config = session.config

        settings.output_dir.mkdir(parents=True, exist_ok=True)

        path = unique_output_path(
            settings.output_dir, f".{settings.file_format}",
        )

        recording = Recording(
            path,
            config,
            device_id=session.device_id,
            background=background,
        )

        clock_text = self._sync_radar_clock(session)

        # The radar delivers int16 samples.
        frame_bytes = (
            config.sweep_count * len(config.traces) * config.sweep_points * 2
        )

        free_bytes = shutil.disk_usage(settings.output_dir).free

        self._post(
            "log",
            f"Recording to {path}\n"
            f"  {format_bytes(frame_bytes)} of raw data per frame, written "
            f"to {recording.part_path.name} until the recording is saved; "
            f"{format_bytes(free_bytes)} free on the disk.\n"
            f"  {clock_text}",
        )

        self._post("log", self._stage_log_status(settings))

        self._post("recording_started", str(path), settings.frames_to_record)

        return ActiveRecording(
            recording=recording,
            frames_limit=settings.frames_to_record,
        )

    def _sync_radar_clock(self, session):
        """Set the radar clock to the PC clock; returns a line for the log."""

        try:
            sync_radar_clock(session.device)

        except Exception as error:
            return (
                f"Warning: the radar clock could not be set to the PC clock "
                f"({type(error).__name__}: {error}); the recording uses the "
                f"clock set at Start."
            )

        return "Radar clock set to the PC clock (UTC) for this recording."

    @staticmethod
    def _stage_log_folders(settings):
        """Where actuator_logger.m writes: Results, or the save folder."""

        return list(dict.fromkeys((settings.output_dir, DEFAULT_OUTPUT_DIR)))

    def _stage_log_status(self, settings):

        try:
            return logger_status(self._stage_log_folders(settings))

        except Exception as error:
            return (
                f"The stage position log could not be checked: "
                f"{type(error).__name__}: {error}"
            )

    def _report_stage_positions(self, folders, span):
        """Saver thread: which stage log covers the recording (span)."""

        if span is None:
            return

        try:
            lines = recording_report(folders, *span)

        except Exception as error:
            lines = [
                f"The stage position log could not be checked: "
                f"{type(error).__name__}: {error}"
            ]

        self._post("log", "\n".join(lines))

    def _record(self, active, frame):
        """Add a frame to the recording; True when the recording is done."""

        recording = active.recording
        recording.add_frame(frame)

        now = time.monotonic()

        done = 0 < active.frames_limit <= recording.n_frames

        if now >= active.next_disk_check:

            active.next_disk_check = now + DISK_CHECK_INTERVAL_S

            free_bytes = shutil.disk_usage(recording.path.parent).free

            if free_bytes < recording.n_bytes + MIN_FREE_DISK_BYTES:

                self._post(
                    "log",
                    f"Only {format_bytes(free_bytes)} of disk space are "
                    f"left, too little to also save the recording; the "
                    f"recording is stopped.",
                )

                done = True

        if done or now >= active.next_status:

            active.next_status = now + STATUS_INTERVAL_S

            self._post(
                "recording",
                RecordingStatus(
                    n_frames=recording.n_frames,
                    frames_limit=active.frames_limit,
                    duration_s=recording.duration_s,
                    n_bytes=recording.n_bytes,
                ),
            )

        return done

    def _finish_recording(self, session, active):
        """Write the recording file in a thread; the radar keeps running."""

        recording = active.recording

        self._post("recording_saving", recording.n_frames, recording.n_bytes)

        saver = threading.Thread(
            target=self._save_recording,
            args=(recording, self._stage_log_folders(session.settings)),
            name="recording-saver",
            daemon=True,
        )
        saver.start()

        session.savers.append(saver)

    def _save_recording(self, recording, stage_log_folders):
        """Saver thread; reports errors instead of raising them."""

        span = recording.sweep_time_range_s

        try:
            path = recording.close()

        except Exception as error:

            self._post(
                "error",
                f"Saving the recording failed: {type(error).__name__}: "
                f"{error}\n\nThe recorded samples are kept in "
                f"{recording.part_path}.",
                traceback.format_exc()
                + f"\nRaw data: {recording.describe_part()}",
            )
            self._post(
                "recording_ended",
                None,
                recording.n_frames,
                recording.duration_s,
                0,
            )
            self._report_stage_positions(stage_log_folders, span)
            return

        for note in recording.notes:
            self._post("log", note)

        if path is None:
            self._post("log", "No frame was recorded, so nothing was saved.")
            self._post("recording_ended", None, 0, 0.0, 0)
            return

        self._post(
            "recording_ended",
            str(path),
            recording.n_frames,
            recording.duration_s,
            path.stat().st_size,
        )

        self._report_stage_positions(stage_log_folders, span)

    def _run_save(self, session):

        background = None

        if session.settings.background_frames:
            background = self._capture_background(session)

        active = self._start_recording(session, background)

        try:

            while not self._stop_event.is_set():

                if self._record(active, self._acquire(session)):
                    break

        finally:
            self._finish_recording(session, active)

    def _run_display(self, session):

        started = time.perf_counter()
        frame = self._acquire(session)
        acquisition_ms = (time.perf_counter() - started) * 1e3

        self._post(
            "display_init",
            DisplaySetup(
                run_id=session.run_id,
                config=session.config,
                trace_map=frame.trace_map,
                full_scale=frame.full_scale,
                settings=session.settings,
            ),
        )

        background = None
        averager = None
        active = None

        frame_number = 0
        frame_times = deque(maxlen=RATE_HISTORY_LENGTH)
        next_progress = 0.0

        try:

            while True:

                frame_number += 1

                now = time.perf_counter()
                frame_times.append(now)

                rate_hz = 0.0

                if len(frame_times) >= 2 and now > frame_times[0]:
                    rate_hz = (len(frame_times) - 1) / (now - frame_times[0])

                # A recording started now begins with the next frame: the
                # radar clock was set again, after this frame was taken.
                recording_started = False

                for command, value in self._take_commands():

                    if command == "record" and value and active is None:

                        try:
                            active = self._start_recording(session, background)
                            recording_started = True

                        except OSError as error:
                            self._post(
                                "error",
                                f"The recording could not be started: "
                                f"{error}",
                                traceback.format_exc(),
                            )
                            self._post("recording_ended", None, 0, 0.0, 0)

                    elif command == "record" and not value and active is not None:
                        self._finish_recording(session, active)
                        active = None

                    elif command == "background":
                        averager = BackgroundAverager(value)
                        self._post(
                            "log", f"Capturing the background ({value} frames)...",
                        )

                if averager is not None:

                    if averager.add(frame):
                        background = averager.result()
                        averager = None
                        self._post("background", background)

                    elif now >= next_progress:
                        next_progress = now + STATUS_INTERVAL_S
                        self._post(
                            "background_progress",
                            averager.count,
                            averager.n_frames,
                        )

                # Saved first, then shown.
                if (
                    active is not None
                    and not recording_started
                    and self._record(active, frame)
                ):
                    self._finish_recording(session, active)
                    active = None

                self._publish_display(
                    DisplayFrame(
                        run_id=session.run_id,
                        frame=frame,
                        frame_number=frame_number,
                        rate_hz=rate_hz,
                        acquisition_ms=acquisition_ms,
                    )
                )

                if self._stop_event.is_set():
                    break

                started = time.perf_counter()
                frame = self._acquire(session)
                acquisition_ms = (time.perf_counter() - started) * 1e3

        finally:

            if active is not None:
                self._finish_recording(session, active)


def main():

    enable_dpi_awareness()

    root = tk.Tk()
    RadarGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
