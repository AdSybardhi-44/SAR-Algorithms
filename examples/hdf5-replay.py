#!/usr/bin/env python3

from twopilabs.sense.x1000 import SenseX1000
from twopilabs.utils.scpi import *
import logging
import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal.windows as windows
import scipy.constants as const
import signal
import argparse
import time
import yaml
from datetime import datetime, timezone
from twopilabs.utils.scpi import ScpiUsbTmcTransport, ScpiTcpIpTransport

try:
    import h5py
except ImportError:
    raise RuntimeError('This example requires the h5py package to be installed')

DEFAULT_VERBOSE = 0
DEFAULT_FILENAME = None #'2piSENSE-X1000-2022-11-03T12_29_57.h5'

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

    argparser = argparse.ArgumentParser(description="Replay a recorded hdf5 file from a Sense X1000 radar device")
    argparser.add_argument("-v",        dest="verbose",     action="count",         default=DEFAULT_VERBOSE,    help="output verbose logging information (Can be specified multiple times)")
    argparser.add_argument("--file",    dest="filename",    metavar="FILENAME",     default=DEFAULT_FILENAME,   type=str, help="HDF5 filename", required=DEFAULT_FILENAME is None)
    args = argparser.parse_args()

    # Set up logging as requested by number of -v switches
    loglevel = {0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG}
    logging.basicConfig(stream=sys.stderr, level=loglevel[args.verbose], format='%(asctime)s %(levelname)-8s %(message)s')
    logger.setLevel(logging.INFO)

    with h5py.File(args.filename, 'r') as h5file:
        # Select the first items in the first two hierarchies
        h5supergroup = list(h5file.values())[0]
        h5group = list(h5supergroup.values())[0]
        logger.info(f'Opening hdf5 group: {h5group.name}')

        logger.info(f'Device information:')
        info = yaml.load(h5group.attrs['info'], Loader=yaml.SafeLoader)
        logger.info(f'  HWTYPE: {info["HWTYPE"]}')
        logger.info(f'  HWREVISION: {info["HWREVISION"]}')
        logger.info(f'  ID: {info["ID"]}')
        logger.info(f'  POWER:CURRENT: {info["POWER"]["CURRENT"]}')
        logger.info(f'  POWER:VOLTAGE: {info["POWER"]["VOLTAGE"]}')
        logger.info(f'  POWER:SOURCE: {info["POWER"]["SOURCE"]}')
        logger.info(f'  ROSCILLATOR:DCTCXO: {info["ROSCILLATOR"]["DCTCXO"]}')
        logger.info(f'  ROSCILLATOR:ENABLED: {info["ROSCILLATOR"]["ENABLED"]}')
        logger.info(f'  ROSCILLATOR:HOLDOVER: {info["ROSCILLATOR"]["HOLDOVER"]}')
        logger.info(f'  ROSCILLATOR:LOCK: {info["ROSCILLATOR"]["LOCK"]}')
        logger.info(f'  ROSCILLATOR:SOURCE: {info["ROSCILLATOR"]["SOURCE"]}')
        logger.info(f'  TEMP: {info["TEMP"]}')
        logger.info(f'  USB: {info["USB"]}')

        for item in h5group.keys():
            if close:
                break

            start = time.time()

            # This generates an acquisition data object from the HDF5 parent group given a HDF5 child group name
            logger.info(f'Replaying group {item}')
            data = SenseX1000.AcqData.from_hdf5(h5group, item)

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
