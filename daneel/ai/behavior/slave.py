import math

import ivy_robot
from behavior import Behavior
import time

DIRECT_SPEED_COMMAND_LINEAR_VALUE = 100.0  # mm/s
DIRECT_SPEED_COMMAND_ROTATION_VALUE = 1.0  # rad/s

class Slave(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.robot.locomotion.x = 1500
        self.robot.locomotion.y = 1000
        if self.robot.ivy is None:
            raise BaseException("No ivy component set on the robot !\n"
                                "Hint : do 'robot.ivy = ivy_robot.Ivy(robot, 192.168.255.255:2010)")
        self.robot.ivy.register_callback(ivy_robot.GO_TO_ORIENT_REGEXP, self.go_to_orient)
        self.robot.ivy.register_callback(ivy_robot.GO_TO_REGEXP, self.go_to)
        self.robot.ivy.register_callback(ivy_robot.CUSTOM_ACTION_REGEXP, self.handle_custom_action)
        self.robot.ivy.register_callback(ivy_robot.SPEED_DIRECTION_REGEXP, self.handle_ivy_speed_direction)

        self._voltage_toggle_time = 0

        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_SIGNAL, self.robot.io.SensorState.PERIODIC)
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_POWER, self.robot.io.SensorState.PERIODIC)

    def loop(self):
        if self.robot.io.battery_power_voltage is not None and self.robot.io.battery_signal_voltage is not None:
            if (self._voltage_toggle_time % 13) < 5:
                self.robot.io.score_display_number(round(self.robot.io.battery_signal_voltage * 100), with_two_points=True)
                self.robot.io.set_led_color(self.robot.io.LedColor.CYAN)
            elif (self._voltage_toggle_time % 13) < 10:
                self.robot.io.score_display_number(round(self.robot.io.battery_power_voltage * 100), with_two_points=True)
                self.robot.io.set_led_color(self.robot.io.LedColor.PURPLE)
            else:
                self.robot.io.score_display_fat()
                self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
            self._voltage_toggle_time = time.time()

    def go_to_orient(self, agent, *arg):
        x, y, theta = arg[0].split(",")
        self.robot.locomotion.navigate_to(float(x), float(y), float(theta))

    def go_to(self, agent, *arg):
        x, y = arg[0].split(",")
        self.robot.locomotion.navigate_to(float(x), float(y), self.robot.locomotion.theta)

    def handle_custom_action(self, agent, *arg):
        custom_action_number = int(arg[0])
        print(custom_action_number)
        if custom_action_number == 1:
            print("action #1 !")
        elif custom_action_number == 2:
            print("action #2 !")

    def handle_ivy_speed_direction(self, agent, *arg):
        x, y, theta = map(lambda f: float(f), arg[0].split(","))
        if x == 0 and y == 0:
            x_speed = 0
            y_speed = 0
        else:
            x_speed = x * DIRECT_SPEED_COMMAND_LINEAR_VALUE  #/ math.hypot(x, y)
            y_speed = y * DIRECT_SPEED_COMMAND_LINEAR_VALUE  #/ math.hypot(x, y)
        theta_speed = theta * DIRECT_SPEED_COMMAND_ROTATION_VALUE
        self.robot.locomotion.set_direct_speed(x_speed, y_speed, theta_speed)

