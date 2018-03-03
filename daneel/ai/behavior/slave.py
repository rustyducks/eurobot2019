import ivy_robot
from behavior import Behavior


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

        #to remove
        self.arm_hand = True
        self.robot.communication.send_actuator_command(3, 512)
        self.robot.communication.send_actuator_command(2, 200)



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
