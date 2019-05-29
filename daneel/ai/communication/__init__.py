"""
Created on 29 July 2012
@author: Guilhem Buisan
"""

import time
from multiprocessing import Process, Queue

import serial

from communication.message_definition import *

SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
SERIAL_SEND_TIMEOUT = 500  # ms
MAX_SEND_RETRIES = 100


def serial_read_loop(serial_path, baudrate, mailbox, sendbox, is_sent_flag):
    tr = TeensyReaderProcess(serial_path, baudrate, mailbox, sendbox, is_sent_flag)
    while True:
        tr.loop()
        time.sleep(0.0001)


class Communication:
    """
    Class handling communication between ai and the Teensy.
    """

    def __init__(self, serial_path=SERIAL_PATH, baudrate=SERIAL_BAUDRATE):
        """
        ctor of the communication class

        :param serial_path: The path of the serial file
        :type serial_path: str
        :param baudrate: The baudrate of UART (must the same as the one on the other board)
        :type baudrate: int
        """
        self._mailbox = Queue()
        self._sendbox = Queue(1)  # We want to be blocked until the message is sent (or do we ?)
        self._is_sent = Queue(1)  # Has to be empty when something needs to be sent, and will be filled when the message is sent
        self.mock_communication = False  # Set to True if Serial is not plugged to the Teensy
        self._callbacks = {msg_type: [] for msg_type in eTypeUp}
        if not self.mock_communication:
            self.reader_process = Process(target=serial_read_loop, args=(serial_path, baudrate, self._mailbox,
                                                                         self._sendbox, self._is_sent),
                                          name="TeensyCommunication")
            self.reader_process.daemon = True
        self.eTypeUp = eTypeUp  # For exposure purposes

    def start(self):
        if not self.mock_communication:
            self.reader_process.start()
            self.reset_soft_teensy()

    def register_callback(self, message_type, callback):
        """
        Use this function to register a function which will be called when a certain message type will
        be received from the Teensy.
        The type of the callbacks must be:

        ============ =============
        message_type callback type
        ============ =============
        ODOM_REPORT  (previous_report_id [int], new_report_id [int], dx [float], dy [float], dtheta [float]) -> void
        HMI_STATE    (cord_state [bool], button1_state [bool], button2_state [bool], red_led_state [bool],
                     green_led_state [bool], blue_led_state [bool]) -> void
        SENSOR_VALUE (sensor_id, sensor_value) -> void
        ============ =============

        :param message_type: The type of the message, which, when received, will trigger the callback.
        :type message_type: eTypeUp
        :param callback: Function which will be called.
        :type callback: function
        """
        if message_type not in self._callbacks:
            return
        self._callbacks[message_type].append(callback)

    def send_speed_command(self, vx, vy, vtheta, max_retries=1000):
        """
        Used to send a speed command to the teensy. The command must be in table frame ! (for vx and vtheta constant,
        the robot makes a straight line while rotating on itself and does not make a circle (which it would do if it was
        in robot frame.))

        :param vx: speed along the table x axis.
        :type vx: float
        :param vy: speed along the table y axis.
        :type vy: float
        :param vtheta: rotation speed (direct with z ascending)
        :type vtheta: float
        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        msg = sMessageDown()
        msg.type = eTypeDown.SPEED_COMMAND
        msg.data = sSpeedCommand()
        msg.data.vx = vx
        msg.data.vy = vy
        msg.data.vtheta = vtheta
        return self.send_message(msg, max_retries)

    def send_hmi_command(self, red_led_cmd, green_led_cmd, blue_led_cmd, max_retries=1000):
        """
        /!\\ Blocking command (try to send the message until it has been received or max_retries)
        Send an HMI (LED) command to the teensy.

        :param red_led_cmd: Red value (0 - 255, then casted to 3 bits)
        :type red_led_cmd: int
        :param green_led_cmd: Green value (0 - 255, then casted to 3 bits)
        :type green_led_cmd: int
        :param blue_led_cmd: Blue value (0 - 255, then casted to 2 bits)
        :type blue_led_cmd: int
        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if message has been sent, -1 if max retries has been reached
        :rtype: int
        """
        msg = sMessageDown()
        msg.type = eTypeDown.HMI_COMMAND
        msg.data = sHMICommand()
        msg.data.hmi_command = (red_led_cmd & 0b11100000) | (green_led_cmd >> 3 & 0b00011100) | (blue_led_cmd >> 6
                                                                                                 & 0b00000011)
        return self.send_message(msg, max_retries)

    def send_actuator_command(self, actuator_id, actuator_value, max_retries=1000):
        """
        Send an actuator command to the Teensy.

        :param actuator_id: The actuator id as defined in base/code/InputOutputs.h:eMsgActuatorId. (0-255)
        :type actuator_id: int
        :param actuator_value: The value of the actuator. Behavior depends on the actuator and what is set
            in the base/code/InputOutputs.cpp:handleActuatorMessage function. (0-65535)
        :type actuator_value: int
        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        msg = sMessageDown()
        msg.type = eTypeDown.ACTUATOR_COMMAND
        msg.data = sActuatorCommand()
        msg.data.actuator_id = actuator_id
        msg.data.actuator_command = actuator_value
        return self.send_message(msg, max_retries)

    def send_sensor_command(self, sensor_id, command_state, max_retries=1000):
        """
        Change a sensor state by sending a command to Teensy.

        :param sensor_id: The sensor id as defined in the base/code/InputOutputs.h:sensors array (which is filled
            in base/code/InputOutputs.h:initSensors function).
        :type sensor_id: int
        :param command_state: The state to enable on the sensor. As defined in
            base/code/InputOutputs.h:sSensor::eSensorReadState.
        :type command_state: int
        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        msg = sMessageDown()
        msg.type = eTypeDown.SENSOR_COMMAND
        msg.data = sSensorCommand()
        msg.data.sensor_id = sensor_id
        msg.data.sensor_state = command_state
        return self.send_message(msg, max_retries)

    def reset_soft_teensy(self, max_retries=1000):
        """
        Send a reset order to the Teensy. This message will be accepted by the Teensy whatever the id (so even
        after a desynchronisation between Teensy and ai, eg. when the ai reboots and not the Teensy)

        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        if self.mock_communication:
            print("[Communication] Warning : Teensy communication mocked !")
            return 0
        msg = sMessageDown()
        msg.type = eTypeDown.RESET
        ret = self.send_message(msg, max_retries)
        if ret == 0:
            while not self._mailbox.empty():
                self._mailbox.get()
        return ret

    def send_repositioning(self, x, y, theta, max_retries=1000):
        """
        Send an angular repositioning command to the Teensy,

        :param x: the x coordinate of the robot
        :type x: float
        :param y: the y coordinate of the robot
        :type y: float
        :param theta: the angle of the robot (direct around an ascending z axis, and from x table axis.)
        :type theta: float
        :param max_retries: number of times to retry if the sending fails (default = 1000)
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        msg = sMessageDown()
        msg.type = eTypeDown.REPOSITIONING
        msg.data = sRepositioning()
        msg.data.x_repositioning = x
        msg.data.y_repositioning = y
        msg.data.theta_repositioning = theta
        return self.send_message(msg, max_retries)

    def send_pid_tuning(self, kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular, max_retries=1000):
        msg = sMessageDown()
        msg.type = eTypeDown.PID_TUNING
        msg.data = sPIDTuning()
        msg.data.kp_linear = kp_linear
        msg.data.ki_linear = ki_linear
        msg.data.kd_linear = kd_linear
        msg.data.kp_angular = kp_angular
        msg.data.ki_angular = ki_angular
        msg.data.kd_angular = kd_angular
        return self.send_message(msg, max_retries)

    def send_message(self, msg, _):
        """
        Send message via Serial (defined during the instantiation of the class)

        :param msg: the message to send
        :type msg: sMessageDown
        :param _: Not used only for legacy purpose
        :type _:
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        if not self._is_sent.empty():
            # This should not happen
            self._is_sent.get()

        self._sendbox.put(msg)
        while self._is_sent.empty():
            time.sleep(0.00001)
        ret = self._is_sent.get()
        return ret

    def check_message(self, max_read=1):
        """
        Check if there is any incoming message on the Serial (defined during the instantiation of the class)
        and returns the oldest message.

        :return: The oldest message non read
        :rtype: sMessageUp
        """

        for i in range(max_read):
            if not self._mailbox.empty():
                msg = self._mailbox.get()
                self.handle_message(msg)

    def handle_message(self, message):
        """
        Call registered callbacks with well formed arguments depending on message.type.

        :param message: The message to be handled (containing the type and the arguments to be passed
            to the callback.
        :type message: sMessageUp
        """
        if message.type == eTypeUp.SENSOR_VALUE:
            for cb in self._callbacks[eTypeUp.SENSOR_VALUE]:
                cb(message.data.sensor_id, message.data.sensor_value)
        elif message.type == eTypeUp.HMI_STATE:
            cord_state = bool(message.data.hmi_state & (1 << 7))
            button1_state = bool(message.data.hmi_state & (1 << 6))
            button2_state = bool(message.data.hmi_state & (1 << 5))
            red_led_state = 255 if bool(message.data.hmi_state & (1 << 4)) else 0
            green_led_state = 255 if bool(message.data.hmi_state & (1 << 3)) else 0
            blue_led_state = 255 if bool(message.data.hmi_state & (1 << 2)) else 0
            for cb in self._callbacks[eTypeUp.HMI_STATE]:
                cb(cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state)
        elif message.type == eTypeUp.ODOM_REPORT:
            for cb in self._callbacks[eTypeUp.ODOM_REPORT]:
                cb(message.data.x, message.data.y, message.data.theta)
        elif message.type == eTypeUp.SPEED_REPORT:
            for cb in self._callbacks[eTypeUp.SPEED_REPORT]:
                cb(message.data.vx, message.data.vy, message.data.vtheta, *message.data.drifting)
        elif message.type == eTypeUp.ACK_DOWN:
            pass


class TeensyReaderProcess:
    STATE_IDLE = 0
    STATE_HEADER_RECEIVED = 1

    def __init__(self, serial_path, baudrate,  mailbox: Queue, sendbox: Queue, is_sent_flag: Queue):
        self._receivebox = mailbox
        self._sendbox = sendbox
        self._is_sent_flag = is_sent_flag
        self._serial_port = serial.Serial(serial_path, baudrate)
        self._serial_port.reset_input_buffer()
        self._tmp_msg = sMessageUp()
        self._state = self.STATE_IDLE
        self._last_start_byte = None
        self._current_msg_id = 0

    def loop(self):
        if not self._sendbox.empty() and self._is_sent_flag.empty():
            msg_to_send = self._sendbox.get()
            if msg_to_send.type == eTypeDown.RESET:
                for i in range(10):
                    time.sleep(0.01)
                    self._serial_port.read_all()
                    self._current_msg_id = 0
            ret = self._send_message(msg_to_send)
            self._is_sent_flag.put(ret, True)
        msg = self._read_message()
        if msg is not None and msg.type != eTypeUp.ACK_DOWN:
            self._receivebox.put(msg)

    def _send_message(self, msg):
        msg.down_id = self._current_msg_id
        self._current_msg_id = (self._current_msg_id + 1) % 256
        serialized = msg.serialize().tobytes()
        for i in range(MAX_SEND_RETRIES):
            # print(serialized)
            self._serial_port.write(b'\xff\xff' + serialized)
            # print("Sending :", b'\xff\xff' + serialized)
            time_sent = int(round(time.time() * 1000))
            while int(round(time.time() * 1000)) - time_sent < SERIAL_SEND_TIMEOUT:
                msg = self._read_message()
                if msg is not None:
                    if msg.type == eTypeUp.ACK_DOWN:
                        return 0  # success
                    else:
                        self._receivebox.put(msg)  # if it is not an ACK or a NONACK, store it to deliver later
        return -1  # failure

    def _read_message(self, max_read=1):
        for i in range(max_read):
            if self._state == self.STATE_IDLE:
                if self._last_start_byte is None and self._serial_port.in_waiting >= 1:
                    self._last_start_byte = self._serial_port.read(1)[0]
                if self._serial_port.in_waiting >= UP_HEADER_SIZE + 1:
                    start_byte = self._serial_port.read(1)[0]
                    if self._last_start_byte == 0xFF and start_byte == 0xFF:
                        try:
                            packed_header = self._serial_port.read(UP_HEADER_SIZE)
                            self._last_start_byte = None
                            self._tmp_msg.deserialize_header(packed_header)
                        except DeserializationException as e:
                            print("[Comm] Message synchronisation lost : Trying to re synchronise : {}".format(e))
                            continue
                        else:
                            self._state = self.STATE_HEADER_RECEIVED
                    else:
                        self._last_start_byte = start_byte

            if self._state == self.STATE_HEADER_RECEIVED:
                if self._serial_port.in_waiting >= self._tmp_msg.data_size:
                    self._state = self.STATE_IDLE
                    packed_data = self._serial_port.read(self._tmp_msg.data_size)
                    msg = sMessageUp()
                    try:
                        msg.up_id = self._tmp_msg.up_id
                        msg.type = self._tmp_msg.type
                        msg.data_size = self._tmp_msg.data_size
                        msg.checksum = self._tmp_msg.checksum
                        msg.deserialize_data(packed_data)
                    except DeserializationException as e:
                        print("[Comm] Cannot deserialize data : {}".format(e))
                        continue
                    self._handle_acknowledgement(msg)
                    return msg

    def _handle_acknowledgement(self, msg):
        if msg.type == eTypeUp.ACK_DOWN:
            return
        else:
            self._send_acknowledgment(msg.up_id)

    def _send_acknowledgment(self, id_to_acknowledge):
        ack = sMessageDown()
        ack.down_id = self._current_msg_id
        self._current_msg_id = (self._current_msg_id + 1) % 256
        ack.type = eTypeDown.ACK_UP
        ack.data = sAckUp()
        ack.data.ack_up_id = id_to_acknowledge
        serialized = ack.serialize().tobytes()
        self._serial_port.write(b'\xff\xff' + serialized)
        # print("Sending :", b'\xff\xff' + serialized)
