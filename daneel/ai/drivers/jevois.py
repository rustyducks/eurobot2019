import serial
import threading
import re
from typing import NamedTuple
import time

DEFAULT_SERIAL_PATH = "/dev/jevois_serial"
DEFAULT_BAUDRATE = 115200

ARMOTHY_BASE_HEIGHT = 200

class JeVois:
    JEVOIS_FOV = 60  # deg
    JEVOIS_RES = (640, 480)

    def __init__(self, serial_path=DEFAULT_SERIAL_PATH, baudrate=DEFAULT_BAUDRATE):
        self._serial = serial.Serial(serial_path, baudrate)
        self._puck_mutex = threading.Lock()
        self._puck_mutex.acquire()
        self._last_pucks = {"red": [], "green": [], "blue": []}
        self._read_number = 0
        self._puck_mutex.release()
        self.rt_thread = threading.Thread(target=self._get_pucks)
#        self._serial.readline()
        self._serial.flush()
        self.rt_thread.start()
    
    def send_pos(self, z, y_rotation):
        command = "armothy_pos {} {}\n".format(int(ARMOTHY_BASE_HEIGHT - z), int(y_rotation))
        self._serial.write(command.encode())

    def _get_pucks(self):
        # x y r mean color;x...
        colors = [[], [], []]
        id_of_col = {"red":0, "green":1, "blue":2}
        while True:
            colors = [[], [], []]
            line = self._serial.readline().decode().strip()
            self._read_number += 1
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

    def get_next_pucks(self):
        i = self._read_number
        while self._read_number == i:
            time.sleep(0.00000001)
        return self.last_pucks


    @property
    def uniq_last_puck_color(self):
        len_red = len(self._last_pucks['red'])
        len_green = len(self._last_pucks['green'])
        len_blue = len(self._last_pucks['blue'])

        if len_red == 0 and len_green == 0 and len_blue == 0:
            return None
        if len_red > 0 and len_green == 0 and len_blue == 0:
            return 'red'
        if len_red == 0 and len_green > 0 and len_blue == 0:
            return 'green'
        if len_red == 0 and len_green == 0 and len_blue > 0:
            return 'blue'
        # at this point, there is more than one atom. Do you want the color of the closest atom? Yes.

        closest_sq_distance_to_center = float('inf')
        closest_color = None
        for r in self._last_pucks['red']:
            d = (r[0] - JeVois.JEVOIS_RES[0]/2)**2 + (r[1] - JeVois.JEVOIS_RES[1]/2) ** 2
            if d < closest_sq_distance_to_center:
                closest_sq_distance_to_center = d
                closest_color = "red"
        for g in self._last_pucks['green']:
            d = (g[0] - JeVois.JEVOIS_RES[0] / 2) ** 2 + (g[1] - JeVois.JEVOIS_RES[1] / 2) ** 2
            if d < closest_sq_distance_to_center:
                closest_sq_distance_to_center = d
                closest_color = "green"
        for b in self._last_pucks['blue']:
            d = (b[0] - JeVois.JEVOIS_RES[0] / 2) ** 2 + (b[1] - JeVois.JEVOIS_RES[1] / 2) ** 2
            if d < closest_sq_distance_to_center:
                closest_sq_distance_to_center = d
                closest_color = "blue"
        return closest_color

    @property
    def closest_puck_to_center(self):
        pucks = self.last_pucks
        closest_sq_distance_to_center = float('inf')
        closest_puck = None
        for r in pucks['red']:
            d = (r[0] - JeVois.JEVOIS_RES[0]/2)**2 + (r[1] - JeVois.JEVOIS_RES[1]/2)**2
            if d < closest_sq_distance_to_center:
                closest_puck = (r[0], r[1], 'red')
                closest_sq_distance_to_center = d
        for g in pucks['green']:
            d = (g[0] - JeVois.JEVOIS_RES[0]/2)**2 + (g[1] - JeVois.JEVOIS_RES[1]/2)**2
            if d < closest_sq_distance_to_center:
                closest_puck = (g[0], g[1], 'green')
                closest_sq_distance_to_center = d
        for b in pucks['blue']:
            d = (b[0] - JeVois.JEVOIS_RES[0]/2)**2 + (b[1] - JeVois.JEVOIS_RES[1]/2)**2
            if d < closest_sq_distance_to_center:
                closest_puck = (b[0], b[1], 'blue')
                closest_sq_distance_to_center = d
        return closest_puck

    @property
    def closest_puck_to_robot(self):
        pucks = self.get_next_pucks()
        min_y = float('inf')
        closest_puck = None
        for color, pucks_list in pucks.items():
            print(color, pucks_list)
            for p in pucks_list:
                d = 510 - p[1]
                if d < min_y:
                    min_y = d
                    closest_puck = (p[0], p[1], color)
        return closest_puck










