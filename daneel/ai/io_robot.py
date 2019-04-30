"""
Created on 29 July 2012
@author: Guilhem Buisan
"""

from enum import *
import threading, serial

import math

from drivers.neato_xv11_lidar import lidar_points, read_v_2_4

LIDAR_SERIAL_PATH = "/dev/ttyUSB0"
LIDAR_SERIAL_BAUDRATE = 115200

BIT10_TO_BATTERY_FACTOR = 0.014774881516587679


class ActuatorID(Enum):
    VL6180X_LEFT_RESET = 0
    VL6180X_CENTER_RESET = 1
    VL6180X_RIGHT_RESET = 2
    SCORE_COUNTER = 4


class IO(object):
    def __init__(self, robot):
        self.robot = robot
        self.cord_state = None
        self.button1_state = None
        self.button2_state = None
        self.led_color = None
        self.score_display_text = None
        self.battery_power_voltage = None
        self.battery_signal_voltage = None
 #       self.lidar_serial = serial.Serial(LIDAR_SERIAL_PATH, LIDAR_SERIAL_BAUDRATE)
 #       self.lidar_thread = threading.Thread(target=read_v_2_4, args=(self.lidar_serial,))
 #       self.lidar_thread.start()
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.SENSOR_VALUE, self._on_sensor_value_receive)

        self.score_display_fat()

    @property
    def lidar_points(self):
        return lidar_points[:] # returns a copy of the lidar point to avoid modification while iterating over the array

    class SensorId(Enum):
        BATTERY_SIGNAL = 0
        BATTERY_POWER = 1

    class SensorState(Enum):
        STOPPED = 0
        ON_CHANGE = 1
        PERIODIC = 2

    def change_sensor_read_state(self, sensor_id: SensorId, sensor_state: SensorState):
        return self.robot.communication.send_sensor_command(sensor_id.value, sensor_state.value)

    class LedColor(Enum):
        BLACK = (0, 0, 0)
        RED = (255, 0, 0)
        GREEN = (0, 255, 0)
        BLUE = (0, 0, 255)
        YELLOW = (255, 255, 0)
        PURPLE = (255, 0, 255)
        CYAN = (0, 255, 255)
        ORANGE = (255, 127, 0)
        WHITE = (200, 200, 200)

    class CordState(Enum):
        IN = "in"
        OUT = "out"

    class ButtonState(Enum):
        PRESSED = "pressed"
        RELEASED = "released"

    class ScoreDisplayTexts(Enum):  # As defined in base/InputOutputs.cpp/InputOutputs::handleActuatorMessage
        ENAC = 20001
        FAT = 20002

    def set_led_color(self, color):
        if isinstance(color, self.LedColor):
            color = color.value
        if self.robot.communication.send_hmi_command(*color) == 0:
            self.led_color = color
            if __debug__:
                print("[IO] Led switched to {}".format(color))

    def score_display_fat(self):
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, self.ScoreDisplayTexts.FAT.value) == 0:
            self.score_display_text = "FAT"
            print("[IO] Score display displays " + self.score_display_text)

    def score_display_enac(self):
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, self.ScoreDisplayTexts.ENAC.value) == 0:
            self.score_display_text = "ENAC"
            print("[IO] Score display displays " + self.score_display_text)

    def score_display_number(self, number, with_two_points=False):
        command = number
        if with_two_points:
            command += 10000
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, command) == 0:
            if with_two_points:
                self.score_display_text = str(number)[:-2] + ":" + str(number)[-2:]
                #FIXME: Not working with number = 4 (text = ":4" instead of ": 4")
            else:
                self.score_display_text = str(number)
            print("[IO] Score display displays " + self.score_display_text)

    def enable_VL6180X_LEFT(self, enable):
        if self.robot.communication.send_actuator_command(ActuatorID.VL6180X_LEFT_RESET.value, int(enable)) == 0:
            pass
            #self.green_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] set VL6180X state to {}".format(enable))

    def enable_VL6180X_CENTER(self, enable):
        if self.robot.communication.send_actuator_command(ActuatorID.VL6180X_CENTER_RESET.value, int(enable)) == 0:
            pass
            #self.green_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] set VL6180X state to {}".format(enable))

    def enable_VL6180X_RIGHT(self, enable):
        if self.robot.communication.send_actuator_command(ActuatorID.VL6180X_RIGHT_RESET.value, int(enable)) == 0:
            pass
            #self.green_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] set VL6180X state to {}".format(enable))

    def _on_hmi_state_receive(self, cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state):
        self.cord_state = self.CordState.IN if cord_state else self.CordState.OUT
        self.button1_state = self.ButtonState.RELEASED if button1_state else self.ButtonState.PRESSED
        self.button2_state = self.ButtonState.RELEASED if button2_state else self.ButtonState.PRESSED
        self.led_color = self.LedColor((red_led_state, green_led_state, blue_led_state))

    def _on_sensor_value_receive(self, sensor_id, sensor_value):
        if sensor_id == self.SensorId.BATTERY_SIGNAL.value:
            self.battery_signal_voltage = self._bit10_to_battery_voltage(sensor_value)
        elif sensor_id == self.SensorId.BATTERY_POWER.value:
            self.battery_power_voltage = self._bit10_to_battery_voltage(sensor_value)

    def _bit10_to_battery_voltage(self, bit10):
        return bit10 * BIT10_TO_BATTERY_FACTOR

    def is_obstacle_in_cone(self, direction, cone_angle, distance):
        direction = round(math.degrees(direction))
        for i in range(len(self.lidar_points)):
            pt = self.lidar_points[i]
            if pt is None:
                return
            a = (pt.azimut - direction + 180) % 360 - 180
            if abs(a) <= cone_angle:

                #print(pt.azimut)
                #print(pt.distance)
                if pt.valid and not pt.warning and pt.distance < distance:
                    x_t = self.robot.locomotion.x + pt.distance * math.cos(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
                    y_t = self.robot.locomotion.y + pt.distance * math.sin(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
                    if not self.robot.map.lidar_table_bb.contains(x_t, y_t):
                        continue
                    in_mask = False
                    for mask in self.robot.map.lidar_static_obstacles_bb:
                        if mask.contains(x_t, y_t):
                            self.robot.ivy.highlight_point(50, x_t, y_t)
                            in_mask = True
                            break
                    if in_mask:
                        continue
                    self.robot.ivy.highlight_point(51, x_t, y_t)
                    # print(x_t, y_t)
                    # print(pt.azimut, pt.distance)
                    return True
        return False


# class USReader(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)
#         # self.i2c = smbus.SMBus(1)
#
#     def run(self):
#         global us_sensors, us_sensors_distance
#         while True:
#             for i, sensor in enumerate(us_sensors):
#                 self.i2c.write_byte_data(sensor.address, 0, 81)
#             time.sleep(0.070)
#             for i, sensor in enumerate(us_sensors):
#                 dst = self.i2c.read_word_data(sensor.address, 2) / 255
#                 if dst != 0:
#                     us_sensors_distance[us_sensors[i]] = dst
