"""
Created on 29 July 2012
@author: Guilhem Buisan
"""

from enum import *
import threading, serial

import math

from drivers.neato_xv11_lidar import lidar_points, read_v_2_4
from drivers import vl6180x as v
from drivers import jevois
import armothy

LIDAR_SERIAL_PATH = "/dev/ttyUSB0"
LIDAR_SERIAL_BAUDRATE = 115200
JEVOIS_SERIAL_PATH = "/dev/jevois_serial"
JEVOIS_SERIAL_BAUDRATE = 115200

BIT10_TO_BATTERY_FACTOR = 0.018


class ActuatorID(Enum):
    VL6180X_LEFT_RESET = 0
    VL6180X_CENTER_RESET = 1
    VL6180X_RIGHT_RESET = 2
    LIDAR_SPEED = 3
    SCORE_COUNTER = 4
    EXPERIMENT_LAUNCHER = 5


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
        self.range_left = v.VL6180X()
        self.range_center = v.VL6180X()
        self.range_right = v.VL6180X()
        self.armothy = armothy.Armothy()
        self.lidar_serial = serial.Serial(LIDAR_SERIAL_PATH, LIDAR_SERIAL_BAUDRATE)
        self.lidar_thread = threading.Thread(target=read_v_2_4, args=(self.lidar_serial,))
        self.lidar_thread.start()
        self.jevois = jevois.JeVois(JEVOIS_SERIAL_PATH, JEVOIS_SERIAL_BAUDRATE)
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.SENSOR_VALUE, self._on_sensor_value_receive)

        self.score_display_rusty_ducks()
        self.set_lidar_pwm(0) # 244 for a good speed  #TODO put a good value in order to use the lidar!
        self.init_range_sensors()

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
        RUSTY_DUCKS = 20003
    
    def init_range_sensors(self):
        self.enable_VL6180X_LEFT(True)
        self.enable_VL6180X_CENTER(False)
        self.enable_VL6180X_RIGHT(False)
        self.range_left.get_ready(0X28)
        self.enable_VL6180X_CENTER(True)
        self.range_center.get_ready(0X30)
        self.enable_VL6180X_RIGHT(True)
        self.range_right.get_ready(0X32)
    
    def do_range_left(self):
        return self.range_left.do_single_shot()
    
    def do_range_center(self):
        return self.range_center.do_single_shot()
    
    def do_range_right(self):
        return self.range_right.do_single_shot()
    
    def set_lidar_pwm(self, pwm):
        pwm=min(abs(int(pwm)), 255)
        if self.robot.communication.send_actuator_command(ActuatorID.LIDAR_SPEED.value, pwm) == 0:
            pass
            if __debug__:
                print("[IO] set lidar pwm to {}".format(pwm))
    
    def launch_experiment(self):
        if self.robot.communication.send_actuator_command(ActuatorID.EXPERIMENT_LAUNCHER.value, 1) == 0:
            pass
            if __debug__:
                print("[IO] Experiment launched!")

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

    def score_display_rusty_ducks(self):
        if self.score_display_text == "Rusty Ducks":
            return
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, self.ScoreDisplayTexts.RUSTY_DUCKS.value) == 0:
            self.score_display_text = "Rusty Ducks"
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
        self.button2_state = self.ButtonState.RELEASED if not button2_state else self.ButtonState.PRESSED
        if red_led_state == green_led_state == blue_led_state == 255:
            self.led_color = self.LedColor.WHITE
        else:
            self.led_color = self.LedColor((red_led_state, green_led_state, blue_led_state))

    def _on_sensor_value_receive(self, sensor_id, sensor_value):
        if sensor_id == self.SensorId.BATTERY_SIGNAL.value:
            self.battery_signal_voltage = self._bit10_to_battery_voltage(sensor_value)
        elif sensor_id == self.SensorId.BATTERY_POWER.value:
            self.battery_power_voltage = self._bit10_to_battery_voltage(sensor_value)

    def _bit10_to_battery_voltage(self, bit10):
        return bit10 * BIT10_TO_BATTERY_FACTOR

    def in_lidar_mask(self, pt):
        x_t = self.robot.locomotion.x + pt.distance * math.cos(
            math.radians(pt.azimut) + self.robot.locomotion.theta)
        y_t = self.robot.locomotion.y + pt.distance * math.sin(
            math.radians(pt.azimut) + self.robot.locomotion.theta)
        if not self.robot.map.lidar_table_bb.contains(x_t, y_t):
            return True
        in_mask = False
        for mask in self.robot.map.lidar_static_obstacles_bb:
            if mask.contains(x_t, y_t):
                self.robot.ivy.highlight_point(50, x_t, y_t)
                in_mask = True
                break
        return in_mask

    def distance_to_cone_ellipse(self, direction, cone_angle, semi_major, semi_minor):
        lidar_points = self.lidar_points
        start_index = round(math.degrees(direction - cone_angle / 2)) % len(lidar_points)
        stop_index = round(math.degrees(direction + cone_angle / 2)) % len(lidar_points)
        while stop_index < start_index:
            start_index -= len(lidar_points)

        min_dist = float('inf')
        max_dist = float('-inf')
        for i in range(start_index, stop_index):
            pt = lidar_points[i]
            if pt is not None and pt.valid and not pt.warning and not self.in_lidar_mask(pt):
                r_ellipse = semi_major * semi_minor / math.sqrt((semi_minor**2 - semi_major**2) * math.cos(math.radians(pt.azimut))**2 + semi_major ** 2)
                d = pt.distance - r_ellipse
                min_dist = min(min_dist, d)
                max_dist = max(max_dist, d)
        return min_dist, max_dist

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
                    if self.in_lidar_mask(pt):
                        continue
                    x_t = self.robot.locomotion.x + pt.distance * math.cos(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
                    y_t = self.robot.locomotion.y + pt.distance * math.sin(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
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
