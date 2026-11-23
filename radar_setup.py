# radar_setup.py

"""Radar discovery, configuration, clock and health report."""

import ctypes
import json
import sys
import time
from datetime import datetime, timezone
from pprint import pprint

from twopilabs.sense.x1000 import SenseX1000

from radar_types import RadarConfig


# FILETIME counts 100 ns intervals since 1601-01-01 UTC.
FILETIME_UNIX_EPOCH = 116_444_736_000_000_000


def _precise_system_time_function():

    if sys.platform != "win32":
        return None

    try:
        return ctypes.windll.kernel32.GetSystemTimePreciseAsFileTime
    except (AttributeError, OSError):
        return None


_GET_SYSTEM_TIME_PRECISE = _precise_system_time_function()


def utc_now():
    """
    PC clock as POSIX seconds UTC (Python float).

    On Windows this is the precise system time: time.time() of Python 3.12
    only advances every 15.6 ms there (API guide section 14, item 25), so
    it can be up to that much behind. MATLAB's datetime('now') reads the
    same precise clock (checked to 0.02 ms), so the radar and the stage log
    (actuator_logger.m) share one time base.
    """

    if _GET_SYSTEM_TIME_PRECISE is None:
        return time.time()

    filetime = ctypes.c_uint64()
    _GET_SYSTEM_TIME_PRECISE(ctypes.byref(filetime))

    return (filetime.value - FILETIME_UNIX_EPOCH) / 1e7


def sync_radar_clock(device):
    """
    Set the radar clock to the PC clock (UTC), so that the trigger time
    stamps of the acquisitions are PC time. The radar accepts fractions of
    a second: its sweep timestamps were measured 1 to 5 ms behind the PC
    clock afterwards, drifting about 6 ppm. (SYST:UTC? returns whole
    seconds of a counter that does not show this.) Returns the PC time
    sent.
    """

    pc_time = utc_now()

    # The library only accepts a Python float (it sends 15 digits, 10 us).
    device.system.utc(float(pc_time))

    return pc_time


def setup_radar(
    F_START,
    F_STOP,
    SWEEP_TIME,
    SWEEP_PERIOD,
    SWEEP_COUNT,
    json_filename="radar_health.json",
    timeout=10.0,
    verbose=True,
    TRACES=(0, 1, 2),
):
    """
    Open the first X1000 radar found, apply the sweep configuration and
    return (device, config).

    F_START and F_STOP in Hz (start above stop gives a falling chirp),
    SWEEP_TIME and SWEEP_PERIOD in s, SWEEP_COUNT sweeps per frame and
    TRACES the receive traces to deliver (0 = TRx, 1 and 2 = Rx).

    The radar checks the values itself and raises ScpiErrorException for
    invalid ones. It rounds them (182 GHz is applied as 181.998 GHz for a
    1 ms sweep), so config holds the applied values. The radar clock is
    set to the PC clock (sync_radar_clock), so the trigger timestamps are
    UTC.

    A health report (identity, applied configuration, PLL lock, radar
    state and system information) is written to json_filename.

    Raises RuntimeError if no radar is found or a PLL is not locked.
    The caller closes the device (device.close()).
    """

    resources = SenseX1000.find_devices()

    if not resources:
        raise RuntimeError("No X1000-series radar found.")

    device = SenseX1000.open_device(resources[0], timeout=timeout)

    try:

        device.core.cls()

        # The radar rounds the frequencies for the sweep time in effect
        # when they are set; with the sweep time set first, the applied
        # frequencies do not depend on the previous configuration.
        # The library only accepts Python float and int values.
        device.sense.sweep_time(float(SWEEP_TIME))
        device.sense.frequency_start(float(F_START))
        device.sense.frequency_stop(float(F_STOP))
        device.sense.sweep_period(float(SWEEP_PERIOD))
        device.sense.sweep_count(int(SWEEP_COUNT))
        device.calc.trace_list([int(trace) for trace in TRACES])

        # The trigger timestamps come from the radar clock, which starts
        # at 2000-01-01 after power-up; set it as the 2piSENSE examples do.
        sync_radar_clock(device)

        applied = device.sense.dump()
        traces = tuple(int(trace) for trace in device.calc.trace_list())

        config = RadarConfig(
            frequency_start_hz=float(applied["FREQUENCY"]["START"]),
            frequency_stop_hz=float(applied["FREQUENCY"]["STOP"]),
            sweep_time_s=float(applied["SWEEP"]["TIME"]),
            sweep_period_s=float(applied["SWEEP"]["PERIOD"]),
            sweep_count=int(applied["SWEEP"]["COUNT"]),
            sweep_points=int(applied["SWEEP"]["POINTS"]),
            sample_interval_s=float(applied["SWEEP"]["TINTERVAL"]),
            traces=traces,
        )

        report = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "sense": applied,
            "traces": list(traces),
            **read_health(device),
        }

        with open(json_filename, "w", encoding="utf-8") as file:
            json.dump(report, file, indent=4, default=str)

        if verbose:
            print_health(report)

        # The radar reports no error for an unlocked PLL, but its data
        # would be unusable.
        if not (report["main_pll_locked"] and report["aux_pll_locked"]):
            raise RuntimeError(
                f"The radar PLLs are not locked (main: "
                f"{report['main_pll_locked']}, aux: "
                f"{report['aux_pll_locked']})."
            )

        return device, config

    except BaseException:

        try:
            device.close()
        except Exception:
            pass

        raise


def read_health(device):
    """Identity, PLL lock, radar state and system information."""

    return {
        "idn": device.core.idn(),
        "main_pll_locked": bool(device.control.radar_mainpll_locked()),
        "aux_pll_locked": bool(device.control.radar_auxpll_locked()),
        "radar_state": device.control.radar_state_current().name,
        "device_info": device.get_info(),
    }


def read_error_queue(device):
    """Pop all entries of the radar's SCPI error queue."""

    errors = []
    error = device.check_error()

    while error is not None:
        errors.append(f"{error.code} | {error.description} | {error.info}")
        error = device.check_error()

    return errors


def print_health(report):

    info = report["device_info"]

    print("\n=== RADAR ===")
    print(f"Identity        : {', '.join(report['idn'])}")
    print(
        f"PLL locked      : main {report['main_pll_locked']}, "
        f"aux {report['aux_pll_locked']}"
    )
    print(f"Radar state     : {report['radar_state']}")
    print(f"Temperatures    : {info.get('TEMP')}")
    print(f"Ref. oscillator : {info.get('ROSCILLATOR')}")

    print("\n=== APPLIED CONFIGURATION ===")
    pprint(report["sense"])
    print(f"Traces          : {report['traces']}")

    if "errors" in report:
        print("\n=== SCPI ERROR QUEUE ===")
        print("\n".join(report["errors"]) or "No SCPI errors.")


def check_radar_health(device):
    """
    Print the health of an open radar and return it as a dict
    (troubleshooting helper). "healthy" is True when both PLLs are locked
    and the SCPI error queue was empty.
    """

    report = {
        "sense": device.sense.dump(),
        "traces": device.calc.trace_list(),
        **read_health(device),
        "errors": read_error_queue(device),
    }

    report["healthy"] = (
        report["main_pll_locked"]
        and report["aux_pll_locked"]
        and not report["errors"]
    )

    print_health(report)
    print("\nOverall:", "PASS" if report["healthy"] else "FAIL")

    return report
