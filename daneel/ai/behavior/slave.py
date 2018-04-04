import math

import ivy_robot
from behavior import Behavior

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

        #to remove
        self.arm_hand = True



    def loop(self):
        pass

    def go_to_orient(self, agent, *arg):
        x, y, theta = arg[0].split(",")
        self.robot.locomotion.go_to_orient(float(x), float(y), float(theta))

    def go_to(self, agent, *arg):
        x, y = arg[0].split(",")
        self.robot.locomotion.go_to_orient(float(x), float(y), self.robot.locomotion.theta)

    def toggle_water_cannon(self):
        if self.robot.io.water_cannon_state == self.robot.io.WaterCannonState.STOPPED:
            self.robot.io.start_water_cannon()
        elif self.robot.io.water_cannon_state == self.robot.io.WaterCannonState.FIRING:
            self.robot.io.stop_water_cannon()

    def toggle_water_collector(self):
        if self.robot.io.water_collector_state == self.robot.io.WaterCollectorState.STOPPED:
            self.robot.io.start_water_collector()
        elif self.robot.io.water_collector_state == self.robot.io.WaterCollectorState.ACTIVATED:
            self.robot.io.stop_water_collector()

    def handle_custom_action(self, agent, *arg):
        custom_action_number = int(arg[0])
        print(custom_action_number)
        if custom_action_number == 1:
            self.toggle_water_cannon()
        elif custom_action_number == 2:
            self.toggle_water_collector()
        elif custom_action_number == 3:
            if self.robot.io.arm_gripper_state == self.robot.io.ArmGripperState.CLOSED:
                self.robot.io.open_arm_gripper()
            else:
                self.robot.io.close_arm_gripper()
        elif custom_action_number == 4:
            self.robot.io.move_arm_base(self.robot.io.ArmBaseState.RAISED)
        elif custom_action_number == 5:
            self.robot.io.move_arm_base(self.robot.io.ArmBaseState.MIDDLE)
        elif custom_action_number == 6:
            self.robot.io.move_arm_base(self.robot.io.ArmBaseState.LOWERED)

    def handle_ivy_speed_direction(self, agent, *arg):
        x, y, theta = map(lambda f: float(f), arg[0].split(","))
        if x == 0 and y == 0:
            x_speed = 0
            y_speed = 0
        else:
            x_speed = x * DIRECT_SPEED_COMMAND_LINEAR_VALUE / math.hypot(x, y)
            y_speed = y * DIRECT_SPEED_COMMAND_LINEAR_VALUE / math.hypot(x, y)
        theta_speed = theta * DIRECT_SPEED_COMMAND_ROTATION_VALUE
        self.robot.locomotion.set_direct_speed(x_speed, y_speed, theta_speed)
