import ivy_robot
from behavior import Behavior

SetSpeedREGEXP = "input2ivy RC_4CH (.*)"

class RemoteControlled(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.robot.locomotion.x = 1500
        self.robot.locomotion.y = 1000
        self.robot.ivy.register_callback(SetSpeedREGEXP, self.set_speed)

    def loop(self):
        pass

    def set_speed(self, agent, *arg):
        print(arg[0])
        acid, mode, throttle, vy, vx, w = arg[0].split(" ")
        self.robot.communication.send_speed_command(-2*int(vx), -2*int(vy), -float(w)/100)
