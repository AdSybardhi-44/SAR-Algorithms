#!/usr/bin/env python3

from twopilabs.sense.x1000 import SenseX1000
import logging
import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal.windows as windows
import scipy.constants as const
import signal
import argparse
import time
from datetime import datetime, timezone
from twopilabs.utils.scpi import ScpiUsbTmcTransport, ScpiTcpIpTransport


DEFAULT_TIMEOUT = 5.0
DEFAULT_VERBOSE = 0
DEFAULT_FSTART = 182
DEFAULT_FSTOP = 126
DEFAULT_TSWEEP = 1
DEFAULT_NSWEEPS = 1
DEFAULT_TPERIOD = 20
DEFAULT_SWEEPMODE = SenseX1000.SweepMode.NORMAL.name
DEFAULT_TRIGSOURCE = SenseX1000.TrigSource.IMMEDIATE.name
DEFAULT_TRACES = [0]


close = False

def plot(data: SenseX1000.AcqData):
    ####
    # Process
    ####

    # Normalize data values to fullscale signed N-bit integer
    # At this point data.array is of dimension N_sweeps x N_trace x N_points
    sweep_data = data.array / data.header.acq_dtype.info.mag

    # Processing and plotting below expects a 2D array with data points in last dimension.
    # Thus flatten the trace and sweep dimensions and leave data dimension intact.
    sweep_data = sweep_data.reshape([-1, data.n_points])

    # FFT processing to get range information,
    fft_length = max(8192, data.n_points)
    window_data = windows.hann(data.n_points, sym=False)[np.newaxis,:]
    range_data = np.fft.fftn(sweep_data * window_data, s=[fft_length], axes=[-1])

    # renormalize magnitude to dBFS
    range_data_abs = np.abs(range_data) / np.sum(window_data)  # Renormalize for windowing function
    range_data_db = 20 * np.log10(range_data_abs * 2) # compensate for single-sided spectrum plot below

    if not plt.fignum_exists(1):
        ####
        # Generate axes for plotting
        ####
        sweep_time_axis = np.linspace(data.header.subhdr_time_axis.start, data.header.subhdr_time_axis.stop, data.header.data_points)
        sweep_freq_axis = np.linspace(data.header.subhdr_freq_axis.start, data.header.subhdr_freq_axis.stop, data.header.data_points)

        range_freq_axis = np.linspace(0, 1/abs(data.header.time_step), fft_length, endpoint=False)
        range_time_axis = np.linspace(0, 1/abs(data.header.freq_step), fft_length, endpoint=False)
        range_dist_axis = range_time_axis * const.c / 2

        ####
        # Plot
        ####
        # First call of plot function. Create plots
        fig, (ax_sweep, ax_space) = plt.subplots(2, 1, figsize=(16, 14))
        fig.canvas.manager.set_window_title('Sense X1000 Live Plot (Press "a" for Autoscale)')

        # Connect event handler for closing plot
        def exit(event):
            global close
            close = True

        def keypress(event):
            if event.key == 'a':
                plot.ax_sweep.relim()
                plot.ax_sweep.autoscale(enable=True, axis='y')

        fig.canvas.mpl_connect('close_event', exit)
        fig.canvas.mpl_connect('key_press_event', keypress)

        plt.ion() # Interactive mode on

        # Sweep Domain plot
        ax_sweep.set_xlabel("Sweep Time [ms]")
        ax_sweep.set_xlim(sweep_time_axis[0] * 1E3, sweep_time_axis[-1] * 1E3)
        ax_sweep.set_ylabel("Amplitude in Full-Scale")
        ax_sweep.set_title("Acquired raw IF sweep domain signal")
        ax_sweep.grid()

        # Add second X axis
        ax_sweep_freq = ax_sweep.twiny()
        ax_sweep_freq.set_xlabel("Instantaneous Sweep Frequency [GHz]")
        ax_sweep_freq.set_xlim(round(sweep_freq_axis[0] / 1E9, 1), round(sweep_freq_axis[-1] / 1E9, 1))

        # Spatial Domain plot
        ax_space.set_xlabel("Distance [m]")
        ax_space.set_xlim(range_dist_axis[0], range_dist_axis[fft_length//2-1])
        ax_space.set_ylabel("Magnitude [dBFS]")
        ax_space.set_ylim(-120, 0)
        ax_space.set_title("Fourier Transformed IF signal")
        ax_space.grid()

        # Add second X axis
        ax_range_freq = ax_space.twiny()
        ax_range_freq.set_xlabel("IF Frequency [MHz]")
        ax_range_freq.set_xlim(round(range_freq_axis[0] / 1E6, 1), round(range_freq_axis[fft_length//2-1] / 1E6, 1))

        # Draw and create both plots
        plot.gr_sweep = ax_sweep.plot(sweep_time_axis * 1E3, sweep_data.T)
        plot.gr_space = ax_space.plot(range_dist_axis[0:fft_length//2], range_data_db[:, 0:fft_length//2].T)

        # store away some variables as attributes to this function for later
        plot.fig = fig
        plot.ax_sweep = ax_sweep
        plot.ax_sweep_freq = ax_sweep_freq
        plot.ax_space = ax_space
        plot.ax_range_freq = ax_range_freq

        # Show plot
        fig.tight_layout()
        plt.show()
        plt.pause(0.001) # Seems to be required for MacOS
    else:
        # Plot already exists, update data and redraw
        for i in range(0, len(plot.gr_sweep)):
            plot.gr_sweep[i].set_ydata(sweep_data[i, :])
        for i in range(0, len(plot.gr_space)):
            plot.gr_space[i].set_ydata(range_data_db[i, 0:fft_length//2])
        plot.fig.canvas.draw()
        plot.fig.canvas.flush_events()
        plt.pause(0.001)

def main():
    def exit(signum, frame):
        global close
        close = True

    logger = logging.getLogger(__name__)
    signal.signal(signal.SIGTERM, exit)
    signal.signal(signal.SIGINT, exit)

    argparser = argparse.ArgumentParser(description="Live plot for Sense X1000 radar devices")
    argparser.add_argument("-v",        dest="verbose",     action="count",         default=DEFAULT_VERBOSE,    help="output verbose logging information (Can be specified multiple times)")
    argparser.add_argument("--fstart",  dest="fstart",      metavar="GIGAHERTZ",    default=DEFAULT_FSTART,     type=float, help="Sweep start frequency")
    argparser.add_argument("--fstop",   dest="fstop",       metavar="GIGAHERTZ",    default=DEFAULT_FSTOP,      type=float, help="Sweep stop frequency")
    argparser.add_argument("--tsweep",  dest="tsweep",      metavar="MILLISECONDS", default=DEFAULT_TSWEEP,     type=float, help="Sweep duration")
    argparser.add_argument("--nsweeps", dest="nsweeps",     metavar="NO. SWEEPS",   default=DEFAULT_NSWEEPS,    type=int,   help="Number of sweeps to perform")
    argparser.add_argument("--tperiod", dest="tperiod",     metavar="MILLISECONDS", default=DEFAULT_TPERIOD,    type=float, help="Time period between successive sweeps")
    argparser.add_argument("--traces",  dest="traces",      metavar="TRACE",        default=DEFAULT_TRACES,     type=int, choices=range(0,3), nargs='+', help="A space separated list of traces to acquire")
    argparser.add_argument("--sweepmode", dest="sweepmode", choices=[mode.name for mode in SenseX1000.SweepMode], default=DEFAULT_SWEEPMODE, help="Sweep mode")
    argparser.add_argument("--trigsource", dest="trigsource", choices=[s.name for s in SenseX1000.TrigSource], default=DEFAULT_TRIGSOURCE, help="Trigger Source")
    argparser.add_argument("--timeout", dest="timeout",     metavar="SECONDS",      default=DEFAULT_TIMEOUT,    type=float, help="Communication timeout")
    args = argparser.parse_args()

    # Set up logging as requested by number of -v switches
    loglevel = {0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG}
    logging.basicConfig(stream=sys.stderr, level=loglevel[args.verbose], format='%(asctime)s %(levelname)-8s %(message)s')
    logger.setLevel(logging.INFO)

    # Look for X1000 series devices on all transports
    # For ethernet, this requires either a configured link-local IPv6 address or a zeroconf IPv4 address
    # The device can be set to a static IP address, see package documentation
    # In case of a static IP, use manual ScpiResource instantiation
    devices = SenseX1000.find_devices()
    #devices = SenseX1000.find_devices(transports=[ScpiUsbTmcTransport]) # This searches for USB connected devices only
    #devices = SenseX1000.find_devices(transports=[ScpiTcpIpTransport]) # This searches on the network only
    #devices = [ScpiResource(ScpiTcpIpTransport, '169.254.112.162:5025')] # Use this in case of non-zeroconf IP address

    logger.info('Devices found connected to system:')
    for device in devices:
        logger.info(f'  - {device.resource_name}')

    if len(devices) == 0:
        logger.error('No Sense X1000 devices found')
        return 2

    # Open the first found device with the given communication timeout value
    with SenseX1000.open_device(devices[0], timeout=args.timeout) as device:
        logger.info(f'Connected to SCPI Resource {devices[0].resource_name}')

        # Recall preset and clear registers
        device.core.rst()
        device.core.cls()

        logger.info(f'*IDN?: {device.core.idn()}')

        # Print some device information
        info = device.system.info()
        logger.info(f'HWTYPE: {info["HWTYPE"]}')
        logger.info(f'HWREVISION: {info["HWREVISION"]}')
        logger.info(f'ID: {info["ID"]}')
        logger.info(f'POWER:CURRENT: {info["POWER"]["CURRENT"]}')
        logger.info(f'POWER:VOLTAGE: {info["POWER"]["VOLTAGE"]}')
        logger.info(f'POWER:SOURCE: {info["POWER"]["SOURCE"]}')
        logger.info(f'ROSCILLATOR:DCTCXO: {info["ROSCILLATOR"]["DCTCXO"]}')
        logger.info(f'ROSCILLATOR:ENABLED: {info["ROSCILLATOR"]["ENABLED"]}')
        logger.info(f'ROSCILLATOR:HOLDOVER: {info["ROSCILLATOR"]["HOLDOVER"]}')
        logger.info(f'ROSCILLATOR:LOCK: {info["ROSCILLATOR"]["LOCK"]}')
        logger.info(f'ROSCILLATOR:SOURCE: {info["ROSCILLATOR"]["SOURCE"]}')
        logger.info(f'TEMP: {info["TEMP"]}')
        logger.info(f'USB: {info["USB"]}')

        # Configure radar with given configuration
        device.sense.frequency_start(args.fstart * 1E9)
        device.sense.frequency_stop(args.fstop * 1E9)
        device.sense.sweep_time(args.tsweep * 1E-3)
        device.sense.sweep_count(args.nsweeps)
        device.sense.sweep_period(args.tperiod * 1E-3)
        device.sense.sweep_mode(SenseX1000.SweepMode[args.sweepmode])
        device.trigger.source(SenseX1000.TrigSource[args.trigsource])
        device.calc.trace_list(args.traces)
        device.system.utc(datetime.now(timezone.utc).timestamp())

        # Dump a configuration object with all configured settings
        config = device.sense.dump()
        logger.info(f'Configuration: {config}')

        # Print some useful status information
        logger.info(f'Aux PLL Lock: {device.control.radar_auxpll_locked()}')
        logger.info(f'Main PLL Lock: {device.control.radar_mainpll_locked()}')
        logger.info(f'Ref. Osc. Source: {device.sense.refosc_source_current()}')
        logger.info(f'Ref. Osc. Status: {device.sense.refosc_status()}')

        # Run acqusition/read/plot loop until ctrl+c requested
        while not close:
            start = time.time()
            # Perform sweep
            logger.info('+ Running sweep(s)')

            # This command accomplishes the same as the following (within one round-trip)
            # device.initiate.immediate() # Initiate (arm) the device
            # device.core.wai() # Wait for an event
            # acq = device.calc.data() # Receive acquisition object
            acq = device.initiate.immediate_and_receive()
            logger.info(f'  Acquisition done [Timestamp: {acq.header.trig_timestamp}, Index: {acq.header.acq_index}]')

            # Read all data from this acquisition in one go
            logger.info('  Reading data')
            data = acq.read()

            # Perform processing and plotting
            logger.info('  Plotting')
            plot(data)

            # Measure time taken
            stop = time.time()
            logger.info(f'Time taken: {stop-start:.3f} sec. (FPS: {1/(stop-start):.1f})')


    return 0


if __name__ == "__main__":
    # Call main function above
    sys.exit(main())

