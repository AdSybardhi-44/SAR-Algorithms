# radar_setup.py

import json
from datetime import datetime, timezone
from pprint import pprint

import numpy as np

from radar_types import RadarConfig
from twopilabs.sense.x1000 import SenseX1000


C = 299_792_458.0

# The radar quantizes the sweep (182 GHz is applied as 181.990665 GHz),
# so applied start/stop frequencies only have to match the requested
# values within this fraction of the requested bandwidth.
FREQUENCY_TOLERANCE_FRACTION = 0.01


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
    Configure an X1000 radar and return a configured device object
    together with a RadarConfig instance.

    TRACES selects the receive traces (0, 1 and/or 2) to enable.

    Returns
    -------
    device
        Open radar device ready for acquisitions.

    config : RadarConfig
        Applied configuration and derived parameters.

    Notes
    -----
    Caller is responsible for eventually calling:

        device.close()
    """

    # ==========================================================
    # Input validation
    # ==========================================================

    if F_STOP <= F_START:
        raise ValueError(
            "F_STOP must be greater than F_START."
        )

    if SWEEP_TIME <= 0:
        raise ValueError(
            "SWEEP_TIME must be positive."
        )

    if SWEEP_PERIOD <= 0:
        raise ValueError(
            "SWEEP_PERIOD must be positive."
        )

    if SWEEP_COUNT < 1:
        raise ValueError(
            "SWEEP_COUNT must be >= 1."
        )

    if (
        len(TRACES) == 0
        or len(set(TRACES)) != len(TRACES)
        or any(trace not in (0, 1, 2) for trace in TRACES)
    ):
        raise ValueError(
            "TRACES must be a non-empty list of distinct "
            "trace numbers from 0, 1 and 2."
        )

    requested_traces = sorted(
        int(trace) for trace in TRACES
    )

    # ==========================================================
    # Discover radar
    # ==========================================================

    devices = SenseX1000.find_devices()

    if not devices:
        raise RuntimeError(
            "No X1000-series radar found."
        )

    device = SenseX1000.open_device(
        devices[0],
        timeout=timeout,
    )

    try:

        # ======================================================
        # Clear error queue
        # ======================================================

        device.core.cls()

        # ======================================================
        # Configure radar
        # ======================================================

        device.sense.frequency_start(F_START)
        device.sense.frequency_stop(F_STOP)

        device.sense.sweep_time(SWEEP_TIME)

        device.sense.sweep_period(SWEEP_PERIOD)
        device.sense.sweep_count(SWEEP_COUNT)

        # Enable the requested receiver traces

        device.calc.trace_list(requested_traces)

        enabled_traces = device.calc.trace_list()

        # ======================================================
        # Full readback
        # ======================================================

        applied_config = device.sense.dump()

        f_start = device.sense.frequency_start()
        f_stop = device.sense.frequency_stop()

        n_points = device.sense.sweep_points()

        sweep_time = device.sense.sweep_time()
        sweep_period = device.sense.sweep_period()
        sweep_count = device.sense.sweep_count()

        sample_interval = (
            applied_config["SWEEP"]["TINTERVAL"]
        )

        # ======================================================
        # Derived parameters
        # ======================================================

        bandwidth = f_stop - f_start

        frequency_step = (
            bandwidth / (n_points - 1)
        )

        range_resolution = (
            C / (2.0 * bandwidth)
        )

        max_one_sided_range = (
            C / (4.0 * frequency_step)
        )

        effective_sample_rate = (
            1.0 / sample_interval
        )

        center_frequency = (
            f_start + f_stop
        ) / 2.0

        wavelength = (
            C / center_frequency
        )

        chirp_slope = (
            bandwidth / sweep_time
        )

        expected_points = (
            round(
                sweep_time / sample_interval
            )
            + 1
        )

        reconstructed_sweep_time = (
            (n_points - 1)
            * sample_interval
        )

        timing_consistent = (
            n_points == expected_points
        )

        range_axis_m = (
            np.arange(n_points)
            * range_resolution
        )

        # ======================================================
        # Verify applied frequencies
        # ======================================================

        frequency_tolerance_hz = (
            FREQUENCY_TOLERANCE_FRACTION
            * (F_STOP - F_START)
        )

        frequency_start_match = (
            abs(f_start - F_START) <= frequency_tolerance_hz
        )

        frequency_stop_match = (
            abs(f_stop - F_STOP) <= frequency_tolerance_hz
        )

        traces_match = (
            sorted(enabled_traces) == requested_traces
        )

        # ======================================================
        # Timestamp
        # ======================================================

        setup_timestamp_utc = (
            datetime.now(
                timezone.utc
            ).isoformat()
        )

        # ======================================================
        # Health information
        # ======================================================

        health = {
            "timestamp_utc":
                setup_timestamp_utc,

            "device_info":
                device.get_info(),

            "main_pll_locked":
                bool(
                    device.control.radar_mainpll_locked()
                ),

            "aux_pll_locked":
                bool(
                    device.control.radar_auxpll_locked()
                ),

            "radar_state":
                str(
                    device.control.radar_state_current()
                ),
        }

        # ======================================================
        # Error queue
        # ======================================================

        errors = []

        error = device.check_error()

        while error is not None:

            errors.append(
                {
                    "code":
                        error.code,

                    "description":
                        error.description,

                    "info":
                        error.info,
                }
            )

            error = device.check_error()

        # ======================================================
        # Acquisition ready flag
        # ======================================================

        ready_for_acquisition = (
            health["main_pll_locked"]
            and health["aux_pll_locked"]
            and timing_consistent
            and len(errors) == 0
        )

        # ======================================================
        # Report
        # ======================================================

        report = {

            "requested_parameters": {
                "F_START": F_START,
                "F_STOP": F_STOP,
                "SWEEP_TIME": SWEEP_TIME,
                "SWEEP_PERIOD": SWEEP_PERIOD,
                "SWEEP_COUNT": SWEEP_COUNT,
                "TRACES": requested_traces,
            },

            "applied_parameters": {

                "frequency_start_hz":
                    f_start,

                "frequency_stop_hz":
                    f_stop,

                "sweep_time_s":
                    sweep_time,

                "sweep_period_s":
                    sweep_period,

                "sweep_count":
                    sweep_count,

                "sweep_points":
                    n_points,

                "sample_interval_s":
                    sample_interval,

                "enabled_traces":
                    enabled_traces,
            },

            "derived_parameters": {

                "bandwidth_hz":
                    bandwidth,

                "center_frequency_hz":
                    center_frequency,

                "wavelength_m":
                    wavelength,

                "chirp_slope_hz_per_s":
                    chirp_slope,

                "frequency_step_hz":
                    frequency_step,

                "range_resolution_m":
                    range_resolution,

                "max_one_sided_range_m":
                    max_one_sided_range,

                "effective_sample_rate_hz":
                    effective_sample_rate,
            },

            "consistency_checks": {

                "expected_points":
                    expected_points,

                "actual_points":
                    n_points,

                "timing_consistent":
                    timing_consistent,

                "reconstructed_sweep_time_s":
                    reconstructed_sweep_time,

                "frequency_start_match":
                    frequency_start_match,

                "frequency_stop_match":
                    frequency_stop_match,

                "frequency_tolerance_hz":
                    frequency_tolerance_hz,

                "frequency_start_deviation_hz":
                    f_start - F_START,

                "frequency_stop_deviation_hz":
                    f_stop - F_STOP,

                "traces_match":
                    traces_match,
            },

            "health": {
                **health,
                "ready_for_acquisition":
                    ready_for_acquisition,
            },

            "full_sense_dump":
                applied_config,

            "errors":
                errors,
        }

        with open(
            json_filename,
            "w",
            encoding="utf-8",
        ) as f:

            json.dump(
                report,
                f,
                indent=4,
                default=str,
            )

        # ======================================================
        # Console output
        # ======================================================

        if verbose:

            print("\n=== APPLIED CONFIGURATION ===")
            pprint(applied_config)

            print("\n=== DERIVED PARAMETERS ===")

            print(
                f"Applied bandwidth       : "
                f"{bandwidth/1e9:.6f} GHz"
            )

            print(
                f"Start / stop deviation  : "
                f"{(f_start - F_START)/1e6:+.3f} / "
                f"{(f_stop - F_STOP)/1e6:+.3f} MHz"
            )

            print(
                f"Center frequency        : "
                f"{center_frequency/1e9:.6f} GHz"
            )

            print(
                f"Wavelength              : "
                f"{wavelength*1e3:.6f} mm"
            )

            print(
                f"Chirp slope             : "
                f"{chirp_slope/1e12:.3f} THz/s"
            )

            print(
                f"Frequency step          : "
                f"{frequency_step/1e6:.6f} MHz"
            )

            print(
                f"Range resolution        : "
                f"{range_resolution*1e3:.3f} mm"
            )

            print(
                f"One-sided max range     : "
                f"{max_one_sided_range:.3f} m"
            )

            print(
                f"Delivered sample spacing: "
                f"{sample_interval*1e9:.3f} ns"
            )

            print(
                f"Effective sample rate   : "
                f"{effective_sample_rate/1e6:.3f} MS/s"
            )

            print("\n=== HEALTH ===")

            print(
                f"Main PLL locked : "
                f"{health['main_pll_locked']}"
            )

            print(
                f"Aux PLL locked  : "
                f"{health['aux_pll_locked']}"
            )

            print(
                f"Radar state     : "
                f"{health['radar_state']}"
            )

            print(
                f"Acquisition ready : "
                f"{ready_for_acquisition}"
            )

            print("\n=== ERROR QUEUE ===")

            if not errors:
                print("No SCPI errors")

            else:

                for e in errors:

                    print(
                        e["code"],
                        e["description"],
                        e["info"],
                    )

        # ======================================================
        # Hard validation
        # ======================================================

        if not frequency_start_match:
            raise RuntimeError(
                f"Applied start frequency {f_start/1e9:.6f} GHz "
                f"differs from requested {F_START/1e9:.6f} GHz "
                f"by more than {frequency_tolerance_hz/1e6:.1f} MHz."
            )

        if not frequency_stop_match:
            raise RuntimeError(
                f"Applied stop frequency {f_stop/1e9:.6f} GHz "
                f"differs from requested {F_STOP/1e9:.6f} GHz "
                f"by more than {frequency_tolerance_hz/1e6:.1f} MHz."
            )

        if not traces_match:
            raise RuntimeError(
                f"Enabled traces {enabled_traces} differ "
                f"from requested traces {requested_traces}."
            )

        if not health["main_pll_locked"]:
            raise RuntimeError(
                "Main PLL is not locked."
            )

        if not health["aux_pll_locked"]:
            raise RuntimeError(
                "Aux PLL is not locked."
            )

        if not timing_consistent:
            raise RuntimeError(
                "Sweep timing inconsistency detected."
            )

        if errors:
            raise RuntimeError(
                f"Radar configuration generated "
                f"{len(errors)} SCPI errors."
            )

        # ======================================================
        # Build RadarConfig
        # ======================================================

        config = RadarConfig(

            frequency_start_hz=f_start,
            frequency_stop_hz=f_stop,

            bandwidth_hz=bandwidth,

            sweep_time_s=sweep_time,
            sweep_period_s=sweep_period,
            sweep_count=sweep_count,

            sweep_points=n_points,

            sample_interval_s=sample_interval,
            effective_sample_rate_hz=effective_sample_rate,

            frequency_step_hz=frequency_step,

            range_resolution_m=range_resolution,
            max_one_sided_range_m=max_one_sided_range,

            expected_points=expected_points,
            timing_consistent=timing_consistent,

            center_frequency_hz=center_frequency,
            wavelength_m=wavelength,

            chirp_slope_hz_per_s=chirp_slope,

            range_axis_m=range_axis_m,

            enabled_traces=enabled_traces,

            setup_timestamp_utc=setup_timestamp_utc,

            main_pll_locked=health["main_pll_locked"],
            aux_pll_locked=health["aux_pll_locked"],

            radar_state=health["radar_state"],

            ready_for_acquisition=
                ready_for_acquisition,
        )

        return device, config

    except Exception:

        try:
            device.close()
        except Exception:
            pass

        raise

def check_radar_health(device):
    """
    Check radar health and print the results to the terminal.

    Parameters
    ----------
    device
        Open SenseX1000 device object.

    Returns
    -------
    health : dict
        Health information dictionary.
    """

    print("\n" + "=" * 60)
    print("RADAR HEALTH CHECK")
    print("=" * 60)

    # --------------------------------------------------
    # Device info
    # --------------------------------------------------

    try:
        device_info = device.get_info()

        print("\n=== DEVICE INFO ===")
        pprint(device_info)

    except Exception as e:

        device_info = None

        print("\n=== DEVICE INFO ===")
        print(f"FAILED: {e}")

    # --------------------------------------------------
    # PLL status
    # --------------------------------------------------

    try:
        main_pll_locked = bool(
            device.control.radar_mainpll_locked()
        )

        aux_pll_locked = bool(
            device.control.radar_auxpll_locked()
        )

        print("\n=== PLL STATUS ===")
        print(f"Main PLL Locked : {main_pll_locked}")
        print(f"Aux PLL Locked  : {aux_pll_locked}")

    except Exception as e:

        main_pll_locked = False
        aux_pll_locked = False

        print("\n=== PLL STATUS ===")
        print(f"FAILED: {e}")

    # --------------------------------------------------
    # Radar state
    # --------------------------------------------------

    try:

        radar_state = str(
            device.control.radar_state_current()
        )

        print("\n=== RADAR STATE ===")
        print(radar_state)

    except Exception as e:

        radar_state = "UNKNOWN"

        print("\n=== RADAR STATE ===")
        print(f"FAILED: {e}")

    # --------------------------------------------------
    # Configuration summary
    # --------------------------------------------------

    try:

        print("\n=== RF CONFIGURATION ===")

        f_start = device.sense.frequency_start()
        f_stop = device.sense.frequency_stop()

        sweep_time = device.sense.sweep_time()
        sweep_period = device.sense.sweep_period()
        sweep_count = device.sense.sweep_count()
        sweep_points = device.sense.sweep_points()

        bandwidth = f_stop - f_start

        print(
            f"F_START      : {f_start/1e9:.6f} GHz"
        )

        print(
            f"F_STOP       : {f_stop/1e9:.6f} GHz"
        )

        print(
            f"Bandwidth    : {bandwidth/1e9:.6f} GHz"
        )

        print(
            f"Sweep Time   : {sweep_time*1e6:.3f} us"
        )

        print(
            f"Sweep Period : {sweep_period*1e6:.3f} us"
        )

        print(
            f"Sweep Count  : {sweep_count}"
        )

        print(
            f"Sweep Points : {sweep_points}"
        )

    except Exception as e:

        print("\n=== RF CONFIGURATION ===")
        print(f"FAILED: {e}")

    # --------------------------------------------------
    # Enabled traces
    # --------------------------------------------------

    try:

        traces = device.calc.trace_list()

        print("\n=== ENABLED TRACES ===")
        print(traces)

    except Exception as e:

        traces = None

        print("\n=== ENABLED TRACES ===")
        print(f"FAILED: {e}")

    # --------------------------------------------------
    # SCPI error queue
    # --------------------------------------------------

    errors = []

    print("\n=== SCPI ERROR QUEUE ===")

    try:

        error = device.check_error()

        while error is not None:

            errors.append(
                {
                    "code": error.code,
                    "description": error.description,
                    "info": error.info,
                }
            )

            print(
                f"{error.code} | "
                f"{error.description} | "
                f"{error.info}"
            )

            error = device.check_error()

        if len(errors) == 0:

            print("No SCPI errors.")

    except Exception as e:

        print(f"FAILED: {e}")

    # --------------------------------------------------
    # Final status
    # --------------------------------------------------

    healthy = (
        main_pll_locked
        and aux_pll_locked
        and len(errors) == 0
    )

    print("\n=== OVERALL STATUS ===")

    if healthy:

        print("PASS: Radar appears healthy.")

    else:

        print(
            "FAIL: Radar health check detected issues."
        )

    print("=" * 60)

    return {
        "healthy": healthy,
        "device_info": device_info,
        "main_pll_locked": main_pll_locked,
        "aux_pll_locked": aux_pll_locked,
        "radar_state": radar_state,
        "enabled_traces": traces,
        "errors": errors,
    }