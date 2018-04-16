from enum import *

from drivers.neato_xv11_lidar import lidar_points, read_v_2_4


LIDAR_SERIAL_PATH = "/dev/ttyACM0"
LIDAR_SERIAL_BAUDRATE = 115200

BIT10_TO_BATTERY_FACTOR = 0.014774881516587679


class ActuatorID(Enum):
    WATER_COLLECTOR_GREEN = 0  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_COLLECTOR_ORANGE = 1  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_CANNON_GREEN = 2     # DC motor
    WATER_CANNON_ORANGE = 3    # DC motor
    ARM_BASE = 4         # Dynamixel
    ARM_GRIPPER = 5      # Dynamixel
    SCORE_COUNTER = 6


class IO(object):
    def __init__(self, robot):
        self.robot = robot
        self.cord_state = None
        self.button1_state = None
        self.button2_state = None
        self.led_color = None
        self.green_water_collector_state = None
        self.green_water_cannon_state = None
        self.orange_water_collector_state = None
        self.orange_water_cannon_state = None
        self.arm_base_state = None
        self.arm_gripper_state = None
        self.score_display_text = None
        self.battery_power_voltage = None
        self.battery_signal_voltage = None
        # self.lidar_serial = serial.Serial(LIDAR_SERIAL_PATH, LIDAR_SERIAL_BAUDRATE)
        # self.lidar_thread = threading.Thread(target=read_v_2_4, args=(self.lidar_serial,))
        # self.lidar_thread.start()
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.SENSOR_VALUE, self._on_sensor_value_receive)

        self.stop_green_water_cannon()
        self.stop_green_water_collector()
        self.stop_orange_water_cannon()
        self.stop_orange_water_collector()
        self.move_arm_base(self.ArmBaseState.RAISED)
        self.close_arm_gripper()
        self.score_display_fat()

    @property
    def lidar_points(self):
        return lidar_points[:] # returns a copy of the lidar point to avoid modification while iterating over the array

    class SensorId(Enum):
        BATTERY_SIGNAL = 0
        BATTERY_POWER = 1
        ARM_BASE_ANGLE = 2
        ARM_GRIP_ANGLE = 3

    class SensorState(Enum):
        STOPPED = 0
        ON_CHANGE = 1
        PERIODIC = 2

    def change_sensor_read_state(self, sensor_id: SensorId, sensor_state: SensorState):
        return self.robot.communication.send_sensor_command(sensor_id.value, sensor_state.value)

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

    class ScoreDisplayTexts(Enum):  # As defined in base/InputOutputs.cpp/InputOutputs::handleActuatorMessage
        ENAC = 20001
        FAT = 20002

    def start_green_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 1) == 0:
            self.green_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] Start green water collector")

    def start_orange_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_ORANGE.value, 1) == 0:
            self.orange_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] Start orange water collector")

    def stop_green_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 0) == 0:
            self.green_water_collector_state = self.WaterCollectorState.STOPPED
            if __debug__:
                print("[IO] Stop green water collector")

    def stop_orange_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_ORANGE.value, 0) == 0:
            self.orange_water_collector_state = self.WaterCollectorState.STOPPED
            if __debug__:
                print("[IO] Stop orange water collector")

    def start_green_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 130) == 0:
            self.green_water_cannon_state = self.WaterCannonState.FIRING
            if __debug__:
                print("[IO] Start green water cannon")

    def start_orange_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_ORANGE.value, 130) == 0:
            self.orange_water_cannon_state = self.WaterCannonState.FIRING
            if __debug__:
                print("[IO] Start orange water cannon")

    def stop_green_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 0) == 0:
            self.green_water_cannon_state = self.WaterCannonState.STOPPED
            if __debug__:
                print("[IO] Stop green water cannon")

    def stop_orange_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_ORANGE.value, 0) == 0:
            self.orange_water_cannon_state = self.WaterCannonState.STOPPED
            if __debug__:
                print("[IO] Stop orange water cannon")

    def set_led_color(self, color):
        if self.robot.communication.send_hmi_command(*color.value) == 0:
            self.led_color = color
            if __debug__:
                print("[IO] Led switched to {}".format(color))

    def move_arm_base(self, state: ArmBaseState):
        if state is not None:
            if self.change_sensor_read_state(self.SensorId.ARM_BASE_ANGLE, self.SensorState.PERIODIC) == 0:
                if self.robot.communication.send_actuator_command(ActuatorID.ARM_BASE.value, state.value) == 0:
                    if __debug__:
                        print("[IO] Moved arm base to {}".format(state))
        else:
            raise AttributeError("None argument passed in move arm base !")

    def open_arm_gripper(self):
        if self.change_sensor_read_state(self.SensorId.ARM_GRIP_ANGLE, self.SensorState.PERIODIC) == 0:
            if self.robot.communication.send_actuator_command(ActuatorID.ARM_GRIPPER.value, self.ArmGripperState.OPEN.value) == 0:
                if __debug__:
                    print("[IO] Arm gripper opened")

    def close_arm_gripper(self):
        if self.change_sensor_read_state(self.SensorId.ARM_GRIP_ANGLE, self.SensorState.PERIODIC) == 0:
            if self.robot.communication.send_actuator_command(ActuatorID.ARM_GRIPPER.value, self.ArmGripperState.CLOSED.value) == 0:
                if __debug__:
                    print("[IO] Arm gripper closed")

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
        elif sensor_id == self.SensorId.ARM_BASE_ANGLE.value:
            self.arm_base_state = sensor_value
        elif sensor_id == self.SensorId.ARM_GRIP_ANGLE.value:
            self.arm_gripper_state = sensor_value

    def _bit10_to_battery_voltage(self, bit10):
        return bit10 * BIT10_TO_BATTERY_FACTOR

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
