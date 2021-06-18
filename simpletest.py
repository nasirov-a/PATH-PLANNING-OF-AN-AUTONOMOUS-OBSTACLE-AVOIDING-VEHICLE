
import logging
import sys
import time
import RPi.GPIO as GPIO                    #Import GPIO library

import math

import numpy as np
import sys

from Adafruit_BNO055 import BNO055


bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

status, self_test, error = bno.get_system_status()

if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')


sw, bl, accel, mag, gyro = bno.get_revision()
