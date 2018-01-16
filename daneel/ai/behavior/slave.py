import ivy_robot
from behavior import Behavior


class Slave(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.robot.locomotion.x = 1500
        self.robot.locomotion.y = 1000
        self.robot.ivy.register_callback(ivy_robot.GO_TO_ORIENT_REGEXP, self.go_to_orient)
        self.robot.ivy.register_callback(ivy_robot.GO_TO_REGEXP, self.go_to)

    def loop(self):
        pass

    def go_to_orient(self, agent, *arg):
        x, y, theta = arg[0].split(",")
        self.robot.locomotion.go_to_orient(float(x), float(y), float(theta))

    def go_to(self, agent, *arg):
        x, y = arg[0].split(",")
        self.robot.locomotion.go_to_orient(float(x), float(y), self.robot.locomotion.theta)
