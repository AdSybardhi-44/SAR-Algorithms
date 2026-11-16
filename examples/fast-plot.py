#!/usr/bin/env python3
import sys
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

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

try:
    import h5py
    import yaml
except ImportError:
    logger.info('h5py library not available, data recording not supported')

import pyqtgraph as pg
import numpy as np
import scipy.signal
import scipy.constants as const
import argparse
from datetime import datetime, timezone
import time
from twopilabs.sense.x1000 import SenseX1000
from twopilabs.utils.scpi import ScpiUsbTmcTransport, ScpiTcpIpTransport


DEFAULT_TIMEOUT = 5.0                                       # Communication timeout [s]
DEFAULT_VERBOSE = 0                                         # Verbose level
DEFAULT_FSTART = 126                                        # Start frequency [GHz]
DEFAULT_FSTOP = 182                                         # Stop frequency [GHz]
DEFAULT_TSWEEP = 1.00                                       # Sweep duration [ms]
DEFAULT_NSWEEPS = 1                                         # Number of sweeps [#]
DEFAULT_TPERIOD = 20                                        # Ramp/pulse repetition frequency PRF [ms]
DEFAULT_SWEEPMODE = SenseX1000.SweepMode.NORMAL.name        # Use always same ramp direction
#DEFAULT_SWEEPMODE = SenseX1000.SweepMode.ALTERNATING.name  # Alternating up/down ramping, set NSWEEPS to >2
DEFAULT_TRIGSOURCE = SenseX1000.TrigSource.IMMEDIATE.name   # Internal auto-trigger
#DEFAULT_TRIGSOURCE = SenseX1000.TrigSource.EXTERNAL.name   # External trigger input
DEFAULT_TRACES = [0]                                        # Trace selection, 0 --> center channel
DEFAULT_FFTLEN = 2**14                                      # 0 means no zero-padding
DEFAULT_PLOTSWEEP=True                                      # Enable IF/Sweep domain plot
DEFAULT_PLOTRANGE=True                                      # Enable range domain plot
DEFAULT_PLOTHISTORY=True                                    # Enable range history plot
DEFAULT_PLOTPHASE=True                                      # Enable plotting of phase in range domain
DEFAULT_LOGSCALE=True                                       # Use logarithmic (dB) scale for range plot
DEFAULT_SHOWPEAKS=True                                      # Highlight peaks in range plot
DEFAULT_HISTORYDEPTH=100                                    # History depth for range history plot
DEFAULT_WINDOWNAME='hann'                                   # Name of window function to use for processing
DEFAULT_MAXFPS=40                                           # Maximum FPS
DEFAULT_FILENAME = f'2piSENSE-X1000-{datetime.utcnow().isoformat(timespec="seconds").replace(":", "_")}.h5' # Colons in filenames are not supported on Windows

class FastPlotWindow(QtWidgets.QMainWindow):
    sweep_plot = None
    range_plot = None
    history_plot = None
    device = None
    axes = None
    sweep_curves = None
    range_curves = None
    phase_curves = None
    history_curve = None
    peak_markers = []
    peak_history = None
    fps_average = None
    last_update = None
    color_cycle = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728','#9467bd',
                   '#8c564b', '#e377c2', '#7f7f7f','#bcbd22', '#17becf']
    peak_height = 0.0001        # Minimum height for peak detector [lin, 0.0001 --> -80dB]
    peak_dist = 0.05            # Minimum distance between detected peaks[m]
    peak_start = 0.2            # Start of peak search [m]

    def __init__(self, args, parent=None):
        super(FastPlotWindow, self).__init__(parent)
        self.args = args
        self.peak_history = np.full(self.args.historydepth, np.nan)
        self.h5group = None

        # Create GUI
        self.setWindowTitle('2πLABS Fast Plotting GUI Example')

        # Build GUI with sweep/range plotting regions and FPS display
        self.main_widget = QtWidgets.QWidget()
        self.main_layout = QtWidgets.QVBoxLayout()
        self.plot_canvas = pg.GraphicsLayoutWidget()
        self.fps_label = QtWidgets.QLabel()
        self.main_layout.addWidget(self.plot_canvas)
        self.status_bar = QtWidgets.QStatusBar()
        self.btn_record = QtWidgets.QPushButton('&Record data')
        self.btn_record.setCheckable(True)
        self.status_bar.addPermanentWidget(self.btn_record)
        self.status_bar.addPermanentWidget(self.fps_label)
        self.setStatusBar(self.status_bar)
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.resize(1600, 1000)

        self.btn_record.clicked.connect(self._btn_record_toggled)

        if 'h5py' not in globals():
            self.btn_record.setEnabled(False)

    def showEvent(self, event):
        # When the GUI is loaded, configure radar and start plotting
        if not self._open_radar():
            self.status_bar.showMessage(f'Error opening Radar')
            return

        # Setup plot
        self._setup_plot()

        # Start loop
        update_timer = QtCore.QTimer(self)
        update_timer.start(1000/self.args.maxfps)
        update_timer.timeout.connect(self._update)

        # Start timer for updating the FPS
        fps_timer = QtCore.QTimer(self)
        fps_timer.start(1000)
        fps_timer.timeout.connect(lambda : {self.fps_label.setText(f'FPS: {self.fps_average:.1f}')})

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

        self.device = SenseX1000.open_device(devices[0], timeout=self.args.timeout)
        logger.info(f'Connected to SCPI Resource {devices[0].resource_name}')

        # Recall preset and clear registers
        self.device.core.rst()
        self.device.core.cls()

        self.idn = self.device.core.idn()
        logger.info(f'*IDN?: {self.idn}')

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
        self.device.sense.sweep_mode(SenseX1000.SweepMode[args.sweepmode])
        self.device.trigger.source(SenseX1000.TrigSource[args.trigsource])
        self.device.calc.trace_list(args.traces)
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
        if self.args.plotsweep :
            # Init Sweep Domain Plot (VNA Frequency Domain)
            self.sweep_plot = self.plot_canvas.addPlot(title='2πSENSE Signal Live Plot - Sweep Domain', row=0, col=0)
            self.sweep_plot.addLegend(offset=(70, 30), size=(100, 60))
            self.sweep_plot.setXRange(self.args.fstart, self.args.fstop, padding=0.02)
            self.sweep_plot.setYRange(-0.2, 0.2)
            self.sweep_plot.showGrid(x=True, y=True, alpha=0.5)
            self.sweep_plot.getAxis('bottom').setLabel(text='Frequency (GHz)')
            self.sweep_plot.getAxis('left').setLabel(text='Norm. Amplitude')

        if self.args.plotrange:
            # Init Range Domain Plot (VNA Time Domain)
            self.range_plot = self.plot_canvas.addPlot(title='2πSENSE Signal Live Plot - Range Domain', row=1, col=0)
            self.range_plot.addLegend(offset=(70, 30), size=(100, 60))
            if self.args.logscale:
                self.range_plot.setYRange(-110, -20)  # set axis range
            else:
                self.range_plot.setYRange(0, 0.04)  # set axis range
            self.range_plot.setXRange(0, 0.8, padding=0.02)
            self.range_plot.showGrid(x=True, y=True, alpha=0.5)
            self.range_plot.getAxis('bottom').setLabel(text='Range (m)')
            self.range_plot.getAxis('left').setLabel(text='Amplitude (dBFS)')

            if self.args.plotphase:
                # Create a second ViewBox to the range plot as overlay
                self.phase_plot = pg.ViewBox()
                self.range_plot.scene().addItem(self.phase_plot)
                self.range_plot.getAxis('right').linkToView(self.phase_plot)
                self.range_plot.getAxis('right').setLabel(text='Phase')
                self.range_plot.getAxis('right').setTicks([[(t, f'{t:.0f}°') for t in [-180, -90, 0, 90, 180]]])
                self.range_plot.getAxis('right').setGrid(False)
                self.range_plot.showAxis('right')
                self.phase_plot.setXLink(self.range_plot)
                self.phase_plot.setYRange(-180, 180, padding=0.1)
                self.range_plot.vb.sigResized.connect( # Handle view resizing
                    lambda: self.phase_plot.setGeometry(self.range_plot.vb.sceneBoundingRect()) and
                            self.phase_plot.linkedViewChanged(self.range_plot.vb, self.phase_plot.XAxis))

        if self.args.plothistory:
            self.history_plot = self.plot_canvas.addPlot(title='2πSENSE Signal Live Plot - Range History', row=2, col=0)
            self.history_plot.setXRange(0, self.args.historydepth-1, padding=0.02)
            self.history_plot.showGrid(x=True, y=True, alpha=0.5)
            self.history_plot.getAxis('bottom').setLabel(text='Sweep History')
            self.history_plot.getAxis('left').setLabel(text='Distance (m)')

    def _update(self):
        # Perform FMCW sweep and read all data in one go
        acq = self.device.initiate.immediate_and_receive()
        data = acq.read()

        # Store data if requested
        if self.h5group is not None:
            h5object = data.to_hdf5(self.h5group)

        # Process data
        sweep_data, range_data = self._process_data(data, fft_length=self.args.fftlen, window_name=self.args.windowname)
        range_data_abs, range_data_rad = np.abs(range_data), np.angle(range_data)

        # Generate axes on first iteration
        if self.axes is None:
            self.axes = {
                # Sweep domain axes as reported by radar
                'sweep_freq': data.header.freq_axis,
                'sweep_time': data.header.time_axis,
                # Axes after FFT transform
                'range_freq': np.linspace(0, 1 / abs(data.header.time_step), range_data.shape[-1], endpoint=False),
                'range_time': np.linspace(0, 1 / abs(data.header.freq_step), range_data.shape[-1], endpoint=False)
            }

        # Find some peaks in the signal
        dist_step =  const.c / 2 * (self.axes['range_time'][1] - self.axes['range_time'][0])
        peak_idxs, _ = scipy.signal.find_peaks(range_data_abs[0, 0:range_data.shape[-1] // 2],
                                               height=self.peak_height,
                                               distance=self.peak_dist/dist_step)
        # Only peaks after the start distance offset and visible in range plot
        peak_idxs = peak_idxs[peak_idxs > int(self.peak_start/dist_step)]

        # Refine the position of the first found peak using quadratic interpolation
        peak_idx = peak_idxs[0] if len(peak_idxs) > 0 else np.nan
        if not np.isnan(peak_idx):
            peak_idx = peak_idx + 0.5 * (range_data_abs[0, peak_idx + 1] - range_data_abs[0, peak_idx - 1]) \
                   / (2 * range_data_abs[0, peak_idx] - range_data_abs[0, peak_idx + 1] - range_data_abs[0, peak_idx - 1])

        # Insert refined peak into peak history
        self.peak_history = np.roll(self.peak_history, -1)
        self.peak_history[-1] = peak_idx * dist_step

        # Update all traces for plots that are active
        # -------------------------------------------
        if self.args.plotsweep:
            if self.sweep_curves is None:
                # This creates all curves in the sweep plot, if they don't already exist
                self.sweep_curves = self._create_curves(self.sweep_plot, data)

            for i in range(len(self.sweep_curves)):
                # Plot new data for all curves
                self.sweep_curves[i].setData(self.axes['sweep_freq'] / 1e9, np.real(sweep_data[i, :]))

        if self.args.plotrange:
            if self.range_curves is None:
                # This creates all curves in the range plot, if they don't already exist
                self.range_curves = self._create_curves(self.range_plot, data)

            for i in range(len(self.range_curves)):
                # Plot new data for all curves upto nyquist frequency
                x = self.axes['range_time'] * const.c / 2
                y = 20*np.log10(range_data_abs[i,:]) if self.args.logscale else range_data_abs[i,:]
                self.range_curves[i].setData(x[0:range_data.shape[-1] // 2], y[0:range_data.shape[-1] // 2])

            if self.args.showpeaks:
                x = self.axes['range_time'][peak_idxs] * const.c / 2
                y = 20*np.log10(range_data_abs[0,peak_idxs]) if self.args.logscale else range_data_abs[0,peak_idxs]
                a = np.rad2deg(range_data_rad[0,peak_idxs])

                # remove and delete all markers
                for peak_marker in self.peak_markers:
                    self.range_plot.removeItem(peak_marker)
                self.peak_markers.clear()
                # and create new ones
                for i, peak_xy in enumerate(zip(x, y)):
                    arrow = pg.ArrowItem(pos=peak_xy, angle=a[i] - 90, brush='r' if i==0 else 'g')
                    self.range_plot.addItem(arrow)
                    self.peak_markers.append(arrow)

            if self.args.plotphase:
                if self.phase_curves is None:
                    # Creates a single phase curve corresponding to the first range curve
                    self.phase_curves = [pg.PlotCurveItem(pen=pg.mkPen('black', width=0.1))]
                    for curve in self.phase_curves: self.phase_plot.addItem(curve)

                for i in range(len(self.phase_curves)):
                    x = self.axes['range_time'] * const.c / 2
                    y = np.rad2deg(range_data_rad[i, :])
                    self.phase_curves[i].setData(x[0:range_data.shape[-1] // 2], y[0:range_data.shape[-1] // 2])

        if self.args.plothistory:
            # This creates the history curve if it does not already exist
            if self.history_curve is None:
                self.history_curve = self.history_plot.plot(pen=pg.mkPen('r', width=1))

            # Set new history data
            self.history_curve.setData(np.arange(self.args.historydepth), self.peak_history, connect="finite")

        # Calculate framerate and schedule next data update
        self._calculate_framerate()

    def _process_data(self, data: SenseX1000.AcqData, fft_length, window_name):
        # Normalize data values to fullscale signed N-bit integer
        # At this point data.array is of dimension N_sweeps x N_trace x N_points
        sweep_data = data.array / data.header.acq_dtype.info.mag

        # Processing and plotting below expects a 2D array with data points in last dimension.
        # Thus flatten the trace and sweep dimensions and leave data dimension intact.
        sweep_data = sweep_data.reshape([-1, data.n_points])

        # Apply window function
        fft_length = max(fft_length, data.n_points)
        window_data = scipy.signal.windows.get_window(window_name, data.n_points, fftbins=True)  # define "periodic" window
        sweep_data_windowed = sweep_data * window_data * fft_length / np.sum(window_data)

        # FFT processing to get range information
        range_data = np.fft.ifftn(sweep_data_windowed, s=[fft_length], axes=[-1])
        range_data = range_data * np.exp(-1j * np.pi * np.arange(fft_length) * (data.n_points - 1) / fft_length)
        range_data = np.flip(range_data, axis=-1)

        return sweep_data, range_data

    def _create_curves(self, parent: pg.PlotItem, data: SenseX1000.AcqData):
        # Account for sweeps and channels being reduced to a single dimension for plotting
        curve_map = [(seq_num, trace_num) for seq_num in data.seq_nums for trace_num in data.trace_map]
        palette = lambda i : self.color_cycle[i % len(self.color_cycle)]
        return [parent.plot(pen=pg.mkPen(palette(i), width=2), name=f'Sweep {item[0]} Channel {item[1]}',
                            useCache=True, autoDownsample=True, clipToView=True, skipFiniteCheck=True)
                for i, item in enumerate(curve_map)]

    def _calculate_framerate(self):
        timestamp = time.time()
        if self.last_update is not None:
            fps = 1.0 / (timestamp - self.last_update)
            self.fps_average = self.fps_average * 0.95 + fps * 0.05 if self.fps_average is not None else fps
        self.last_update = timestamp

    def _btn_record_toggled(self, checked):
        if checked:
            self.btn_record.setStyleSheet('background-color: red')
            self.h5file = h5py.File(self.args.filename, 'a')

            # Create a simple folder structure in HDF5 file from *IDN? string
            h5groupname = f'/{self.idn[0]}/{self.idn[1]} ({self.idn[2]})'
            logger.info(f'Opening hdf5 group {h5groupname}')
            self.h5group = self.h5file.require_group(h5groupname)

            # Store device info/state as a YAML string attribute
            if 'info' not in self.h5group.attrs:
                self.h5group.attrs.create('info', yaml.dump(self.device.get_info()))
            if 'state' not in self.h5group.attrs:
                self.h5group.attrs.create('state', yaml.dump(self.device.get_state()))

            self.status_bar.showMessage(f'Started data recording into file: {self.h5file}', timeout=5000)
        else:
            self.btn_record.setStyleSheet('')
            self.status_bar.showMessage(f'Stopped data recording into file: {self.h5file}', timeout=5000)
            self.h5group = None
            self.h5file.close()

if __name__ == '__main__':
    # Parse command line arguments using the given DEFAULT values
    argparser = argparse.ArgumentParser(description="Fast plot for Sense X1000 radar devices")
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
    argparser.add_argument("--plotsweep", dest="plotsweep", default=DEFAULT_PLOTSWEEP, type=bool, help="Enable or disable plotting of sweep domain data")
    argparser.add_argument("--plotrange", dest="plotrange", default=DEFAULT_PLOTRANGE, type=bool, help="Enable or disable plotting of range domain data")
    argparser.add_argument("--plothistory", dest="plothistory", default=DEFAULT_PLOTHISTORY, type=bool, help="Enable or disable plotting of range history")
    argparser.add_argument("--plotphase", dest="plotphase", default=DEFAULT_PLOTPHASE, type=bool, help="Enable or disable plotting of phase information in range domain")
    argparser.add_argument("--showpeaks", dest="showpeaks", default=DEFAULT_SHOWPEAKS, type=bool, help="Enable or disable showing peak phase indicators")
    argparser.add_argument("--logscale", dest="logscale",   default=DEFAULT_LOGSCALE, type=bool, help="Enable or disable logarithmic plotting in range domain")
    argparser.add_argument("--fftlen", dest="fftlen",       default=DEFAULT_FFTLEN, type=int, help="Sets the FFT length for range plot")
    argparser.add_argument("--historydepth", dest="historydepth", default=DEFAULT_HISTORYDEPTH, type=int, help="Sets the number of item in range history")
    argparser.add_argument("--windowname", dest="windowname", default=DEFAULT_WINDOWNAME, type=str, help="Selects the window function used for processing")
    argparser.add_argument("--maxfps",  dest="maxfps",      default=DEFAULT_MAXFPS, type=float, help="Limits the maximum framerate")
    argparser.add_argument("--file",    dest="filename",    metavar="FILENAME",     default=DEFAULT_FILENAME,   type=str, help="HDF5 filename for recording")
    args = argparser.parse_args()

    loglevel = {0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG}
    logging.basicConfig(stream=sys.stderr,
                        level=loglevel[args.verbose],
                        format='%(asctime)s %(levelname)-8s %(message)s')


    # Basic styling with white background more suitable for beamer
    white = QtGui.QColor(255, 255, 255)
    black = QtGui.QColor(0,0,0)
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, white)
    palette.setColor(QtGui.QPalette.WindowText, black)
    pg.setConfigOption('background', white)
    pg.setConfigOption('foreground', black)

    # Instantiate application and plot window
    app = QtWidgets.QApplication(sys.argv)
    app.setPalette(palette)
    window = FastPlotWindow(args)
    window.show()
    sys.exit(app.exec_() if 'PySide2' in sys.modules else app.exec())
