from collections import namedtuple
from enum import *
import threading
import serial
import time

from drivers.neato_xv11_lidar import lidar_points, read_v_2_4

# UltraSoundSensor = namedtuple('ultra_sound_sensor', ['address', 'position'])
# us_sensors = [UltraSoundSensor(0x76, "front_left"), UltraSoundSensor(0x77, "front_right"), UltraSoundSensor(0x73, "rear_left"),
#               UltraSoundSensor(0x74, "rear_center"), UltraSoundSensor(0x72, "rear_right")]  # Sets US sensors here !, empty list if no US is plugged
# # us_sensors=[]
# us_sensors_distance = {us_sensors[i]: 0 for i in range(len(us_sensors))}

LIDAR_SERIAL_PATH = "/dev/ttyACM0"
LIDAR_SERIAL_BAUDRATE = 115200

class ActuatorID(Enum):
    WATER_COLLECTOR_GREEN = 0  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_COLLECTOR_ORANGE = 1  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_CANNON_GREEN = 2     # DC motor
    WATER_CANNON_ORANGE = 3    # DC motor
    ARM_BASE = 4         # Dynamixel
    ARM_GRIPPER = 5      # Dynamixel
    SCORE_COUNTER = 6

# def get_us_distance(i):
#     global us_sensors, us_sensors_distance
#     return us_sensors_distance[us_sensors[i]]


class IO(object):
    def __init__(self, robot):
        # self._thread_us_reader = USReader()
        # self._thread_us_reader.start()
        self.robot = robot
        self.cord_state = None
        self.button1_state = None
        self.button2_state = None
        self.led_color = None
        self.water_collector_state = None
        self.water_cannon_state = None
        self.arm_base_state = None
        self.arm_gripper_state = None
        # self.lidar_serial = serial.Serial(LIDAR_SERIAL_PATH, LIDAR_SERIAL_BAUDRATE)
        # self.lidar_thread = threading.Thread(target=read_v_2_4, args=(self.lidar_serial,))
        # self.lidar_thread.start()
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)

        self.stop_water_cannon()
        self.stop_water_collector()
        self.move_arm_base(self.ArmBaseState.RAISED)
        self.close_arm_gripper()

    @property
    def lidar_points(self):
        return lidar_points[:] # returns a copy of the lidar point to avoid modification while iterating over the array

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

    class WaterCollectorState(Enum):
        ACTIVATED = "activated"
        STOPPED = "stopped"

    class WaterCannonState(Enum):
        FIRING = "firing"
        STOPPED = "stopped"

    class ArmBaseState(Enum):
        RAISED = 200
        MIDDLE = 385
        LOWERED = 490

    class ArmGripperState(Enum):
        OPEN = 400
        CLOSED = 518

    # @staticmethod
    # def get_us_distance_by_postion(position):
    #     global us_sensors
    #     correct_sensors = [i for i in range(len(us_sensors)) if position.lower() in us_sensors[i].position.lower()]
    #     distances = [get_us_distance(i) for i in correct_sensors]
    #     if len(distances) == 0:
    #         return 500000
    #     else:
    #         return min(distances)

    # @property
    # def front_distance(self):
    #     return self.get_us_distance_by_postion("front")
    #
    # @property
    # def rear_distance(self):
    #     return self.get_us_distance_by_postion("rear")

    def start_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 1) == 0:
            self.water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] Start water collector")

    def stop_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 0) == 0:
            self.water_collector_state = self.WaterCollectorState.STOPPED
            if __debug__:
                print("[IO] Stop water collector")

    def start_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 180) == 0:
            self.water_cannon_state = self.WaterCannonState.FIRING
            if __debug__:
                print("[IO] Start water cannon")

    def stop_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 0) == 0:
            self.water_cannon_state = self.WaterCannonState.STOPPED
            if __debug__:
                print("[IO] Stop water cannon")

    def set_led_color(self, color):
        if self.robot.communication.send_hmi_command(*color.value) == 0:
            self.led_color = color
            if __debug__:
                print("[IO] Led switched to {}".format(color))

    def move_arm_base(self, state: ArmBaseState):
        if state is not None:
            if self.robot.communication.send_actuator_command(ActuatorID.ARM_BASE.value, state.value) == 0:
                self.arm_base_state = state  # TODO : Update state only when sensor value is sent by teensy
                if __debug__:
                    print("[IO] Moved arm base to {}".format(state))
        else:
            raise AttributeError("None argument passed in move arm base !")

    def open_arm_gripper(self):
        if self.robot.communication.send_actuator_command(ActuatorID.ARM_GRIPPER.value, self.ArmGripperState.OPEN.value) == 0:
            self.arm_gripper_state = self.ArmGripperState.OPEN  # TODO : Update state only when sensor value is sent by teensy
            if __debug__:
                print("[IO] Arm gripper opened")

    def close_arm_gripper(self):
        if self.robot.communication.send_actuator_command(ActuatorID.ARM_GRIPPER.value, self.ArmGripperState.CLOSED.value) == 0:
            self.arm_gripper_state = self.ArmGripperState.CLOSED # TODO : Update state only when sensor value is sent by teensy
            if __debug__:
                print("[IO] Arm gripper closed")

    def _on_hmi_state_receive(self, cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state):
        self.cord_state = self.CordState.IN if cord_state else self.CordState.OUT
        self.button1_state = self.ButtonState.RELEASED if button1_state else self.ButtonState.PRESSED
        self.button2_state = self.ButtonState.RELEASED if button2_state else self.ButtonState.PRESSED
        self.led_color = self.LedColor((red_led_state, green_led_state, blue_led_state))

    def is_obstacle_in_cone(self, direction, cone_angle, distance):
        for i in range(len(self.lidar_points)):
            pt = self.lidar_points[i]
            a = (pt.azimut - direction + 180) % 360 - 180
            if abs(a) <= cone_angle:
                print(pt.azimut)
                print(pt.distance)
                if pt.valid and not pt.warning and pt.distance < distance:
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
