#!/usr/bin/env python3

from twopilabs.sense.x1000 import SenseX1000
from twopilabs.utils.scpi import *
import logging
import sys
import argparse
import os

def main():
    logger = logging.getLogger(__name__)

    argparser = argparse.ArgumentParser(description="Live plot for Sense X1000 radar devices")
    argparser.add_argument("-v",        dest="verbose",     action="count",         default=0,                  help="output verbose logging information (Can be specified multiple times)")
    argparser.add_argument("-q",        dest="quiet",       action='store_true',    default=False,              help="Don't ask before programming")
    argparser.add_argument("file",      type=argparse.FileType('rb'), help="firmware image file")
    
    args = argparser.parse_args()

    # Set up logging as requested by number of -v switches
    loglevel = {0: logging.WARNING, 1: logging.INFO, 2: logging.DEBUG}
    logging.basicConfig(stream=sys.stderr, level=loglevel[args.verbose], format='%(asctime)s %(levelname)-8s %(message)s')
    logger.setLevel(logging.INFO)

    # Look for X1000 series devices
    devices = SenseX1000.find_devices()

    logger.info('Devices found connected to system:')
    for device in devices:
        logger.info(f'  - {device.resource_name}')

    if len(devices) == 0:
        logger.error('No Sense X1000 devices found')
        return 2

    with SenseX1000.open_device(devices[0], timeout=30.0) as device:
        logger.info(f'Connected to SCPI Resource {devices[0].resource_name}')
        logger.info(f'Firmware version: {device.execute("SYSTEM:INFO:FIRMWARE:VERSION?", result=ScpiString)}')
        logger.info(f'Firmware date: {device.execute("SYSTEM:INFO:FIRMWARE:DATE?", result=ScpiString)}')

        image_length = device.execute('DEVICE:IMAGE:LENGTH?', result=ScpiNumber).as_int()
        block_size = device.execute('DEVICE:IMAGE:BSIZE?', result=ScpiNumber).as_int()
        block_count = image_length/block_size
        idn = (device.execute("*IDN?", result=ScpiString).as_string()).split(',')

        if not args.quiet:
            print(f'Upgrade firmware of {idn[1]} device with serial number {idn[2]} from version {idn[3]}')
            print(f'using file {os.path.basename(args.file.name)}. Press Enter to confirm, Ctrl+C to exit')

            try:
                input()
            except KeyboardInterrupt:
                print('Aborted')
                return -1

        logger.info('Writing firmware file to device...')

        for offset in range(0, image_length, block_size):
            if offset % (image_length/16) == 0:
                # Progress indicator
                logger.info(f'Current block {offset//block_size} of {image_length//block_size}')

            device.execute(f'DEVICE:IMAGE:DATA {offset:d},', param=ScpiArbStream(args.file, block_size))

        logger.info('Preparing firmware update, please wait...')
        device.execute('DEVICE:IMAGE:UPDATE')
        device.raise_error()
        logger.info('The device is rebooting and applying the update, this process can take a few minutes!')

    return 0


if __name__ == "__main__":
    # Call main function above
    sys.exit(main())
