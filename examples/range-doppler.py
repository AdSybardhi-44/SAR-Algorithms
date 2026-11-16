#!/usr/bin/env python3
import sys
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logging.basicConfig(stream=sys.stderr, format='%(asctime)s %(levelname)-8s %(message)s')

try:
    import PySide2
    from PySide2 import QtGui, QtCore, QtWidgets
except ImportError:
    try:
        import PySide6
        from PySide6 import QtGui, QtCore, QtWidgets
    except ImportError:
        logger.error('This example requires PySide2 or PySide6 to be installed')
        sys.exit(100)

import pyqtgraph as pg
import numpy as np
import scipy.signal
import scipy.constants as const
import argparse
from datetime import datetime, timezone
import time
from twopilabs.sense.x1000 import SenseX1000
from twopilabs.utils.scpi import ScpiUsbTmcTransport, ScpiTcpIpTransport


DEFAULT_FSTART = 126                                        # Sweep start frequency [GHz]
DEFAULT_FSTOP = 148.5                                       # Sweep stop frequency [GHz] ## FCC Reduced BW Setting
DEFAULT_TSWEEP = 0.2                                        # Sweep duration [ms]
DEFAULT_NSWEEPS = 100                                       # Number of sweeps per trigger [#]
DEFAULT_TPERIOD = 0.5                                       # Time between sweeps [ms]
DEFAULT_FFTLEN_FT = 2 ** 10                                 # Number of FFT points for fast-time transform
DEFAULT_FFTLEN_ST = 2 ** 8                                  # Number of FFT points for slow-time transform
DEFAULT_WINDOWNAME = 'hann'                                 # Name of window function to use for processing


class RDPlotWindow(QtWidgets.QWidget):
    axes = None
    fps_average = None
    last_update = None
    rd_image = None

    def __init__(self, args, parent=None):
        super(RDPlotWindow, self).__init__(parent)
        self.args = args

        # Create GUI
        self.setWindowTitle('2π-LABS 2D-Range-Doppler Showcase')

        # Build basic GUI
        self.main_layout = QtWidgets.QVBoxLayout()
        self.plot_canvas = pg.GraphicsLayoutWidget()
        self.status_label = QtWidgets.QLabel()
        self.main_layout.addWidget(self.plot_canvas)
        self.main_layout.addWidget(self.status_label)
        self.setLayout(self.main_layout)
        self.resize(1600, 1000)


    def showEvent(self, event):
        # When the GUI is loaded, configure radar and start plotting
        if not self._open_radar():
            self.status_label.setText('Error opening Radar')
            return

        # Setup plot
        self._setup_plot()

        # Start loop
        self._update()

        # Start timer for updating the FPS
        fps_timer = QtCore.QTimer(self)
        fps_timer.start(1000)
        fps_timer.timeout.connect(lambda : {self.status_label.setText(f'FPS: {self.fps_average:.1f}')})

    def _open_radar(self):
        # Look for X1000 series devices
        devices = SenseX1000.find_devices()
        # devices = SenseX1000.find_devices(transports=[ScpiUsbTmcTransport]) # This searches for USB connected devices only
        # devices = SenseX1000.find_devices(transports=[ScpiTcpIpTransport]) # This searches on the network only
        # devices = [ScpiResource(ScpiTcpIpTransport, '169.254.112.162:5025')] # Use this in case of non-zeroconf IP address

        logger.info('Devices found connected to system:')
        for device in devices:
            logger.info(f'  - {device.resource_name}')

        if len(devices) == 0:
            logger.error('No Sense X1000 devices found')
            return False

        self.device = SenseX1000.open_device(devices[0])
        logger.info(f'Connected to SCPI Resource {devices[0].resource_name}')

        # Recall preset and clear registers
        self.device.core.rst()
        self.device.core.cls()

        logger.info(f'*IDN?: {self.device.core.idn()}')

        # Print some device information
        info = self.device.system.info()
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
        self.device.sense.frequency_start(args.fstart * 1E9)
        self.device.sense.frequency_stop(args.fstop * 1E9)
        self.device.sense.sweep_time(args.tsweep * 1E-3)
        self.device.sense.sweep_count(args.nsweeps)
        self.device.sense.sweep_period(args.tperiod * 1E-3)
        self.device.sense.sweep_mode(SenseX1000.SweepMode.NORMAL)
        self.device.calc.trace_list([0])
        self.device.system.utc(datetime.now(timezone.utc).timestamp())

        # Dump a configuration object with all configured settings
        logger.info(f'Configuration: {self.device.sense.dump()}')

        # Print some useful status information
        logger.info(f'Aux PLL Lock: {self.device.control.radar_auxpll_locked()}')
        logger.info(f'Main PLL Lock: {self.device.control.radar_mainpll_locked()}')
        logger.info(f'Ref. Osc. Source: {self.device.sense.refosc_source_current()}')
        logger.info(f'Ref. Osc. Status: {self.device.sense.refosc_status()}')

        return True

    def _setup_plot(self):
        # Create 2D-RD plot object
        self.rd_plot = self.plot_canvas.addPlot(title='2D-Range-Doppler Plot')
        self.rd_lut = pg.HistogramLUTItem()
        self.rd_lut.gradient.loadPreset('yellowy')
        self.rd_lut.setHistogramRange(-130, -70)
        self.rd_lut.setLevels(-110, -80)
        self.plot_canvas.addItem(self.rd_lut)
        self.rd_plot.showAxis('top')
        self.rd_plot.showAxis('right')

        # Generate Plot-Axis
        self.rd_plot.setLabels(left='Recession Velocity (m/s)', bottom='Distance (m)')
        self.rd_plot.showGrid(x=True, y=True)

    def _update(self):
        # Perform FMCW sweep and read all data in one go
        acq = self.device.initiate.immediate_and_receive()
        data = acq.read()

        # Perform processing and plotting
        sweep_data, rd_data = self._process_data(data, fft_lengths=(self.args.stfftlen, self.args.ftfftlen),
                                                 window_name=self.args.windowname)
        # Generate axes on first iteration
        if self.axes is None:
            self.axes = {
                # Fast Time Sweep-Domain Axis Calculation
                'ft_sweep_freq': data.header.freq_axis,
                # Fast Time Range-Domain Axis Calculation
                'ft_range_time': np.linspace(0, 1 / abs(data.header.freq_step), rd_data.shape[1], endpoint=False),
                # Slow Time Sweep-Domain Axis Calculation
                'st_sweep_time': np.array(data.seq_nums) * data.header.sweep_period / np.timedelta64(1, 's'),
                # Slow Time Range-Domain Axis Calculation
                'st_range_freq': np.linspace(-(np.timedelta64(1, 's') / data.header.sweep_period) / 2,
                                              (np.timedelta64(1, 's') / data.header.sweep_period) / 2, rd_data.shape[0], endpoint=False)
            }

        if self.rd_image is None:
            # This defines the extent of the generated dataset
            extent = [ self.axes['ft_range_time'][-1] * const.c / 2,
                       self.axes['st_range_freq'][-1] * const.c / np.mean(data.header.freq_axis) ]

            # This transform sets the correct scaling and position to the image generated range-doppler data
            transform = QtGui.QTransform()
            transform.scale(extent[0] / rd_data.shape[1], extent[1] / rd_data.shape[0])
            transform.translate(0, -rd_data.shape[0] / 2)  # Set the origin to the center of the image on Y axis

            # Generate ImageItem object with axisOrder row-major. Note that 0|0 is in the top-left
            self.rd_image = pg.ImageItem(axisOrder='row-major')
            self.rd_image.setTransform(transform)
            self.rd_lut.setImageItem(self.rd_image)
            self.rd_plot.addItem(self.rd_image)
            self.rd_plot.setRange(xRange=[0, extent[0]/2], yRange=[-extent[1]/2, extent[1]/2], padding=0)

        # Plot range-doppler data, use only positive frequencies for fast-time axis (they are symmetric)
        rd_data = rd_data[:, rd_data.shape[1]//2:]
        self.rd_image.setImage(20*np.log10(np.abs(rd_data)), autoLevels=False)

        # Calculate framerate and schedule next data update
        self._calculate_framerate()
        QtCore.QTimer.singleShot(1, self._update)

    def _process_data(self, data: SenseX1000.AcqData, fft_lengths, window_name):
        # Normalize data values to fullscale signed N-bit integer
        # At this point data.array is of dimension N_sweeps x N_trace x N_points
        sweep_data = data.array / data.header.acq_dtype.info.mag

        # Processing and plotting below expects a 2D array with data points in last dimension.
        # Thus flatten the trace and sweep dimensions and leave data dimension intact.
        sweep_data = sweep_data.reshape([-1, data.n_points])

        # Apply window function and scale amplitude correctly with regards to window size and fft length
        window_data_ft = scipy.signal.windows.get_window(window_name, data.n_points, fftbins=True)  # define "periodic" window
        window_data_st = scipy.signal.windows.get_window(window_name, data.n_sweeps, fftbins=True)  # define "periodic" window
        sweep_data_windowed = sweep_data * np.outer(window_data_st * fft_lengths[0] / np.sum(window_data_st),
                                                    window_data_ft * fft_lengths[1] / np.sum(window_data_ft))

        # Transform using iFFT and shift both axes so that their zero components are in the center
        rd_data = np.fft.ifftn(sweep_data_windowed, s=fft_lengths)
        rd_data = np.fft.fftshift(rd_data)

        return sweep_data, rd_data

    def _calculate_framerate(self):
        timestamp = time.time()
        if self.last_update is not None:
            fps = 1.0 / (timestamp - self.last_update)
            self.fps_average = self.fps_average * 0.95 + fps * 0.05 if self.fps_average is not None else fps
        self.last_update = timestamp


if __name__ == '__main__':
    # Parse command line arguments using the given DEFAULT values
    argparser = argparse.ArgumentParser(description="Range-Doppler Plot for Sense X1000 radar devices")
    argparser.add_argument("-v",        dest="verbose",     action="count",         default=0,                  help="output verbose logging information (Can be specified multiple times)")
    argparser.add_argument("--fstart",  dest="fstart",      metavar="GIGAHERTZ",    default=DEFAULT_FSTART,     type=float, help="Sweep start frequency")
    argparser.add_argument("--fstop",   dest="fstop",       metavar="GIGAHERTZ",    default=DEFAULT_FSTOP,      type=float, help="Sweep stop frequency")
    argparser.add_argument("--tsweep",  dest="tsweep",      metavar="MILLISECONDS", default=DEFAULT_TSWEEP,     type=float, help="Sweep duration")
    argparser.add_argument("--nsweeps", dest="nsweeps",     metavar="NO. SWEEPS",   default=DEFAULT_NSWEEPS,    type=int,   help="Number of sweeps to perform")
    argparser.add_argument("--tperiod", dest="tperiod",     metavar="MILLISECONDS", default=DEFAULT_TPERIOD,    type=float, help="Time period between successive sweeps")
    argparser.add_argument("--stfftlen", dest="stfftlen",   metavar="N",            default=DEFAULT_FFTLEN_ST, type=int, help="Sets the FFT length for slow-time transform")
    argparser.add_argument("--ftfftlen", dest="ftfftlen",   metavar="M",            default=DEFAULT_FFTLEN_FT, type=int, help="Sets the FFT length for fast-time transform")
    argparser.add_argument("--windowname", dest="windowname",                       default=DEFAULT_WINDOWNAME, type=str, help="Selects the window function used for processing")
    args = argparser.parse_args()

    # Set up logging as requested by number of -v switches
    loglevel = {0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG}
    logging.basicConfig(level=loglevel[args.verbose])

    # Basic styling
    white = QtGui.QColor(255, 255, 255)
    darkgrey = QtGui.QColor(53, 53, 53)
    dark_palette = QtGui.QPalette()
    dark_palette.setColor(QtGui.QPalette.Window, darkgrey)
    dark_palette.setColor(QtGui.QPalette.WindowText, white)
    pg.setConfigOption('background', darkgrey)
    pg.setConfigOption('foreground', white)

    # Instantiate application and plot window
    app = QtWidgets.QApplication(sys.argv)
    app.setPalette(dark_palette)
    window = RDPlotWindow(args)
    window.show()
    sys.exit(app.exec_() if 'PySide2' in sys.modules else app.exec())
