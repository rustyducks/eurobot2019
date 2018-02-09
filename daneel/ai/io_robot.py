from collections import namedtuple
from enum import *
import threading
# import smbus
import time

UltraSoundSensor = namedtuple('ultra_sound_sensor', ['address', 'position'])
us_sensors = [UltraSoundSensor(0x76, "front_left"), UltraSoundSensor(0x77, "front_right"), UltraSoundSensor(0x73, "rear_left"),
              UltraSoundSensor(0x74, "rear_center"), UltraSoundSensor(0x72, "rear_right")]  # Sets US sensors here !, empty list if no US is plugged
# us_sensors=[]
us_sensors_distance = {us_sensors[i]: 0 for i in range(len(us_sensors))}


def get_us_distance(i):
    global us_sensors, us_sensors_distance
    return us_sensors_distance[us_sensors[i]]


class IO(object):
    def __init__(self, robot):
        # self._thread_us_reader = USReader()
        # self._thread_us_reader.start()
        self.robot = robot
        self.cord_state = None
        self.button1_state = None
        self.button2_state = None
        self.led_color = None
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)

    class LedColor(Enum):
        BLACK = (False, False, False)
        RED = (True, False, False)
        GREEN = (False, True, False)
        BLUE = (False, False, True)
        YELLOW = (True, True, False)
        PURPLE = (True, False, True)
        CYAN = (False, True, True)
        WHITE = (True, True, True)

    class CordState(Enum):
        IN = "in"
        OUT = "out"

    class ButtonState(Enum):
        PRESSED = "pressed"
        RELEASED = "released"

    @staticmethod
    def get_us_distance_by_postion(position):
        global us_sensors
        correct_sensors = [i for i in range(len(us_sensors)) if position.lower() in us_sensors[i].position.lower()]
        distances = [get_us_distance(i) for i in correct_sensors]
        if len(distances) == 0:
            return 500000
        else:
            return min(distances)

    @property
    def front_distance(self):
        return self.get_us_distance_by_postion("front")

    @property
    def rear_distance(self):
        return self.get_us_distance_by_postion("rear")

    def set_led_color(self, color):
        self.robot.communication.send_hmi_command(*color.value)
        self.led_color = color
        if __debug__:
            print("[IO] Led switched to {}".format(color))

    def _on_hmi_state_receive(self, cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state):
        self.cord_state = self.CordState.IN if cord_state else self.CordState.OUT
        self.button1_state = self.ButtonState.PRESSED if button1_state else self.ButtonState.RELEASED
        self.button2_state = self.ButtonState.PRESSED if button2_state else self.ButtonState.RELEASED
        self.led_color = self.LedColor((red_led_state, green_led_state, blue_led_state))


class USReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # self.i2c = smbus.SMBus(1)

    def run(self):
        global us_sensors, us_sensors_distance
        while True:
            for i, sensor in enumerate(us_sensors):
                self.i2c.write_byte_data(sensor.address, 0, 81)
            time.sleep(0.070)
            for i, sensor in enumerate(us_sensors):
                dst = self.i2c.read_word_data(sensor.address, 2) / 255
                if dst != 0:
                    us_sensors_distance[us_sensors[i]] = dst
