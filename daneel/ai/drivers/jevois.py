import serial
import threading
import re
from typing import NamedTuple

DEFAULT_SERIAL_PATH = "/dev/ttyACM1"
DEFAULT_BAUDRATE = 115200


class JeVois:
    JEVOIS_FOV = 60  # deg
    JEVOIS_RES = (640, 480)

    def __init__(self, serial_path=DEFAULT_SERIAL_PATH, baudrate=DEFAULT_BAUDRATE):
        self._serial = serial.Serial(serial_path, baudrate)
        self._puck_mutex = threading.Lock()
        self._puck_mutex.acquire()
        self._last_pucks = {"red": [], "green": [], "blue": []}
        self._puck_mutex.release()
        self.rt_thread = threading.Thread(target=self._get_pucks)

    def _get_pucks(self):
        # Serial is formatted like so : 408,264:250,125;652,253:  (R:G:B)\n
        colors = [[], [], []]
        line = self._serial.readline()
        for i, color in enumerate(line.split(":")):
            if len(color) != 0:
                for pt in color.split(";"):
                    x, y = pt.split(",")
                    colors[i].append((int(x), int(y)))
        if self._puck_mutex.acquire(blocking=False):
            self._last_pucks['red'] = colors[0]
            self._last_pucks['green'] = colors[1]
            self._last_pucks['blue'] = colors[2]
            self._puck_mutex.release()

    @property
    def last_pucks(self):
        self._puck_mutex.acquire()
        tmp = self._last_pucks.copy()
        self._puck_mutex.release()
        return tmp




