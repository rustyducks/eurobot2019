import sys

import datetime
import argparse
import builtins

import communication
from io_robot import *
from locomotion import *

TRACE_FILE = "/home/pi/code/primary_robot/ai/log/log_"+str(datetime.datetime.now()).replace(' ', '_')

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1
}

robot = None

class Robot(object):
    def __init__(self, behavior=behaviors["FSMMatch"]):
        self.communication = communication.Communication("/dev/ttyS0")
        self.io = IO(self)
        self.locomotion = Locomotion(self)
        if behavior == behaviors["FSMMatch"]:
            from fsmmatch import FSMMatch
            self.behavior = FSMMatch(self)
        elif behavior == behaviors["FSMTests"]:
            raise NotImplementedError("This behavior is not implemented yet !")
        else:
            raise NotImplementedError("This behavior is not implemented yet !")


def main():
    global robot
    robot = Robot()
    # Arguments parsing
    robot.communication.mock_communication = parsed_args.no_teensy
    robot.communication.register_callback(communication.eTypeUp.HMI_STATE, new_hmi_state_callback)
    robot.communication.register_callback(communication.eTypeUp.ODOM_REPORT, robot.locomotion.handle_new_odometry_report)
    robot.communication.register_callback(communication.eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: print(
        "X : {}, Y : {}, Theta : {}".format(robot.locomotion.x, robot.locomotion.y, robot.locomotion.theta)))
    while True:
        msg = robot.communication.check_message()
        robot.behavior.loop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser("AI option parser")
    parser.add_argument('--no_teensy', action='store_true', default=False,
                        help="Mock communications with teensy")
    parsed_args = parser.parse_args()
    if __debug__:
        with open(TRACE_FILE, 'w') as sys.stdout:
            main()
    else:
        main()


def new_hmi_state_callback(cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state):
    if __debug__:
        print("[Comm] New HMI State : cord : {}; b1 : {}; b2 : {}; red LED : {}; green LED : {}; blue LED : {}".format(
            cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state))
    robot.io.cord_state = cord_state
    robot.io.button1_state = button1_state
    robot.io.button2_state = button2_state
