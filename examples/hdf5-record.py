#!/usr/bin/env python3

from twopilabs.sense.x1000 import SenseX1000
from twopilabs.utils.scpi import *
import logging
import sys
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

DEFAULT_TIMEOUT = 5.0
DEFAULT_VERBOSE = 0
DEFAULT_FSTART = 182
DEFAULT_FSTOP = 126
DEFAULT_TSWEEP = 1
DEFAULT_NSWEEPS = 2
DEFAULT_TPERIOD = 10
DEFAULT_SWEEPMODE = SenseX1000.SweepMode.NORMAL.name
DEFAULT_TRIGSOURCE = SenseX1000.TrigSource.IMMEDIATE.name
DEFAULT_TRACES = [0,1,2]
DEFAULT_FILENAME = f'2piSENSE-X1000-{datetime.utcnow().isoformat(timespec="seconds").replace(":", "_")}.h5' # Colons in filenames are not supported on Windows

close = False

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
    argparser.add_argument("--file",    dest="filename",    metavar="FILENAME",     default=DEFAULT_FILENAME,   type=str, help="HDF5 filename")
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
        # Open a new h5py File for saving data into
        with h5py.File(args.filename, 'w') as h5file:
            logger.info(f'Connected to SCPI Resource {devices[0].resource_name}')

            # Recall preset and clear registers
            device.core.rst()
            device.core.cls()

            idn = device.core.idn()
            logger.info(f'*IDN?: {idn}')

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

            # Print some useful status information
            logger.info(f'Aux PLL Lock: {device.control.radar_auxpll_locked()}')
            logger.info(f'Main PLL Lock: {device.control.radar_mainpll_locked()}')
            logger.info(f'Ref. Osc. Source: {device.sense.refosc_source_current()}')
            logger.info(f'Ref. Osc. Status: {device.sense.refosc_status()}')

            # Create a simple folder structure in HDF5 file from *IDN? string
            h5groupname = f'/{idn[0]}/{idn[1]} ({idn[2]})'
            logger.info(f'Opening hdf5 group {h5groupname}')
            h5group = h5file.create_group(h5groupname)

            # Store device info/state as a YAML string attribute
            h5group.attrs.create('info', yaml.dump(device.get_info()))
            h5group.attrs.create('state', yaml.dump(device.get_state()))

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

                # Serialize the data into the HDF5 file, calling without "name" automatically generates a unique name
                h5object = data.to_hdf5(h5group)
                logger.info(f'  Saved as {h5object.name}')

                # Measure time taken
                stop = time.time()
                logger.info(f'Time taken: {stop-start:.3f} sec. (FPS: {1/(stop-start):.1f})')

    return 0


if __name__ == "__main__":
    # Call main function above
    sys.exit(main())
