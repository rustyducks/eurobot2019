import serial
import threading
import re
from typing import NamedTuple

DEFAULT_SERIAL_PATH = "/dev/jevois_serial"
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
#        self._serial.readline()
        self._serial.flush()
        self.rt_thread.start()

    def _get_pucks(self):
        # x y r mean color;x...
        colors = [[], [], []]
        id_of_col = {"red":0, "green":1, "blue":2}
        while True:
            colors = [[], [], []]
            line = self._serial.readline().decode().strip()
            for puck_data in line.split(";"):
                try:
                    x,y,r,mean,col = puck_data.split(" ")
                except ValueError:
                    #print("ValueError", puck_data)
                    continue  #probably partial line (at startup)
                colors[id_of_col[col]].append((int(x),int(y)))
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




