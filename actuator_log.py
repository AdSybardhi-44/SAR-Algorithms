# actuator_log.py

"""
Stage positions logged by actuator_logger.m, for the radar recordings.

actuator_logger.m runs in the MATLAB of the Linear Actuator App (OTA Hub)
and writes Results/actuator_<yyyymmdd_HHMMSS>.csv, one row per reading of
the stage position or event:

    timestamp_s   POSIX seconds UTC of the PC clock: the clock of
                  radar_timestamps in the recordings (radar_gui.py sets
                  the radar clock to the PC clock when a recording starts)
    position_mm   stage position in the coordinates of the app (mm); NaN
                  in an event row
    source        move   read by the app while the stage moves (about
                         every 0.25 s, and when the move ends)
                  halt   where STOP halted the stage
                  idle   read by the logger while the stage stands still
                         (every 0.5 s)
                  event  a message of the stage or of the logger (note)
    segment       coordinate system of the positions; it changes when the
                  position counter is set again (Create stepper, Set home
                  here, Auto home). Positions of different segments must
                  not be mixed.
    utc           timestamp_s as text
    note          the message, or details of the row

An event whose note starts with "Warning:" marks a move that ended without
a final reading: the position is uncertain in the time it names.

The stage position at every sweep of a recording is the linear
interpolation of the positions at radar_timestamps, e.g.

    log = read_actuator_log(path)
    rows = log.positions & (log.segment == segment)
    x_mm = np.interp(radar_timestamps, log.timestamp_s[rows],
                     log.position_mm[rows], left=np.nan, right=np.nan)
"""

import csv
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


LOG_PATTERN = "actuator_*.csv"
LOG_HEADER = ["timestamp_s", "position_mm", "source", "segment", "utc", "note"]

# The logger writes a position at least every 0.5 s while the stage is ready.
ACTIVE_LOG_AGE_S = 5.0

# After a recording, wait this long for the first position after its end.
REPORT_WAIT_S = 3.0


@dataclass(frozen=True)
class ActuatorLog:
    """The rows of a stage position log as arrays (see the module docstring)."""

    path: Path
    timestamp_s: np.ndarray
    position_mm: np.ndarray
    source: np.ndarray
    segment: np.ndarray
    note: np.ndarray

    @property
    def positions(self):
        """True for the rows with a position."""

        return np.isfinite(self.position_mm)


def read_actuator_log(path):
    """
    Read a log written by actuator_logger.m. A row that is still being
    written (the last one, while the logger runs) is skipped.
    """

    path = Path(path)
    rows = []

    with open(path, newline="", encoding="utf-8", errors="replace") as file:

        reader = csv.reader(file)

        if next(reader, None) != LOG_HEADER:
            raise ValueError(f"{path.name} is not a stage position log.")

        for row in reader:

            if len(row) != len(LOG_HEADER):
                continue

            try:
                rows.append(
                    (float(row[0]), float(row[1]), row[2], int(row[3]), row[5])
                )
            except ValueError:
                continue

    columns = list(zip(*rows)) or [()] * 5

    return ActuatorLog(
        path=path,
        timestamp_s=np.array(columns[0], dtype=np.float64),
        position_mm=np.array(columns[1], dtype=np.float64),
        source=np.array(columns[2], dtype=str),
        segment=np.array(columns[3], dtype=np.int64),
        note=np.array(columns[4], dtype=str),
    )


def find_actuator_logs(folders):
    """The stage position logs in folders, oldest first."""

    logs = {}

    for folder in folders:

        folder = Path(folder)

        if not folder.is_dir():
            continue

        for path in folder.glob(LOG_PATTERN):
            try:
                logs[path.resolve()] = path.stat().st_mtime
            except OSError:
                pass

    return sorted(logs, key=logs.get)


def logger_status(folders, now_s=None):
    """One line for the start of a recording: is the stage being logged?"""

    if now_s is None:
        now_s = time.time()

    folders = [Path(folder) for folder in folders]
    logs = find_actuator_logs(folders)
    start_it = (
        "Start actuator_logger in the MATLAB of the Linear Actuator App "
        "(and Connect, Create stepper, Start the stage) to log the stage "
        "positions of this recording."
    )

    if not logs:
        where = " or ".join(str(folder) for folder in dict.fromkeys(folders))
        return f"Warning: no stage position log in {where}. {start_it}"

    path = logs[-1]

    try:
        log = read_actuator_log(path)
    except (OSError, ValueError) as error:
        return f"Warning: the stage position log {path} cannot be read ({error})."

    times = log.timestamp_s[log.positions]

    if times.size and now_s - times[-1] <= ACTIVE_LOG_AGE_S:
        return (
            f"Stage positions are logged to {path} (last position "
            f"{max(now_s - times[-1], 0.0):.1f} s ago)."
        )

    if times.size:
        last = f"last position {now_s - times[-1]:.0f} s ago"
    else:
        last = "no position yet"

    return (
        f"Warning: the stage positions are not being logged ({path.name}: "
        f"{last}). {start_it}"
    )


def recording_report(folders, start_s, end_s, wait_s=REPORT_WAIT_S):
    """
    Lines for the log after a recording whose sweeps span start_s ... end_s
    (POSIX s UTC): which stage log covers it, and what is missing. Waits up
    to wait_s for the first position after end_s.
    """

    deadline = time.monotonic() + wait_s

    while True:

        found = _covering_log(folders, start_s, end_s)

        if found is None or found[1]["after"] or time.monotonic() >= deadline:
            break

        time.sleep(0.25)

    if found is None:
        return [
            f"Warning: no stage position log covers this recording "
            f"({_clock(start_s)} to {_clock(end_s)}): its sweeps have no "
            f"stage position."
        ]

    log, cover = found
    lines = [
        f"Stage positions of the recording: {cover['inside']} readings in "
        f"{log.path}, {cover['low']:.3f} to {cover['high']:.3f} mm (segment "
        f"{', '.join(str(segment) for segment in cover['segments'])})."
    ]

    if not cover["before"]:
        lines.append(
            f"Warning: the stage log starts {cover['first'] - start_s:.1f} s "
            f"after the first sweep: the sweeps before have no stage position."
        )

    if not cover["after"]:
        lines.append(
            f"Warning: the stage log ends {end_s - cover['last']:.1f} s "
            f"before the last sweep: the sweeps after have no stage position."
        )

    if len(cover["segments"]) > 1:
        lines.append(
            "Warning: the stage coordinates changed during the recording "
            "(Create stepper, Set home here or Auto home): the positions "
            "before and after are not comparable."
        )

    if cover["warnings"]:
        lines.append(
            f"Warning: {cover['warnings']} stage move(s) ended without a "
            f"final reading during the recording; the position is uncertain "
            f"there (see the Warning rows of {log.path.name})."
        )

    return lines


def _covering_log(folders, start_s, end_s):
    """The log with the most positions in start_s ... end_s, and its cover."""

    best = None

    for path in find_actuator_logs(folders):

        try:
            # written for the last time before the recording started
            if path.stat().st_mtime < start_s - 1.0:
                continue

            log = read_actuator_log(path)

        except (OSError, ValueError):
            continue

        cover = _cover(log, start_s, end_s)

        if cover is not None and (best is None or cover["inside"] > best[1]["inside"]):
            best = (log, cover)

    return best


def _cover(log, start_s, end_s):

    rows = np.flatnonzero(log.positions)
    times = log.timestamp_s[rows]

    if times.size == 0 or times[-1] < start_s or times[0] > end_s:
        return None

    # The positions that interpolation at start_s ... end_s uses.
    first = max(int(np.searchsorted(times, start_s, side="right")) - 1, 0)
    last = min(int(np.searchsorted(times, end_s, side="left")), times.size - 1)
    used = rows[first:last + 1]

    window = (log.timestamp_s >= times[first]) & (log.timestamp_s <= times[last])

    return {
        "inside": int(np.count_nonzero((times >= start_s) & (times <= end_s))),
        "before": bool(times[0] <= start_s),
        "after": bool(times[-1] >= end_s),
        "first": float(times[0]),
        "last": float(times[-1]),
        "low": float(np.min(log.position_mm[used])),
        "high": float(np.max(log.position_mm[used])),
        "segments": sorted({int(segment) for segment in log.segment[used]}),
        "warnings": int(np.count_nonzero(
            window & np.char.startswith(log.note, "Warning")
        )),
    }


def _clock(posix_s):
    return datetime.fromtimestamp(posix_s).strftime("%H:%M:%S")
