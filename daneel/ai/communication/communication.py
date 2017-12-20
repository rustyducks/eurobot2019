import bitstring
from enum import *
import serial
import time
from collections import deque
from communication.message_definition import UP_MESSAGE_SIZE, sMessageUp, eTypeUp, sMessageDown

from RPi import GPIO

SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
SERIAL_SEND_TIMEOUT = 500  # ms


class Communication:
    def __init__(self, serial_path=SERIAL_PATH, baudrate=SERIAL_BAUDRATE):
        """
        ctor of the communication class
        :param serial_path: The path of the serial file
        :type serial_path: str
        :param baudrate: The baudrate of UART (must the same as the one on the other board)
        :type baudrate: int
        """
        self._serial_port = serial.Serial(serial_path, baudrate)
        self._current_msg_id = 0  # type: int
        self._mailbox = deque()
        self.mock_communication = False  # Set to True if Serial is not plugged to the Teensy

    def send_message(self, msg, max_retries=1000):
        """
        Send message via Serial (defined during the instantiation of the class)
        :param msg: the message to send
        :type msg: sMessageDown
        :param max_retries: the maximum number of resend (on timeout = SERIAL_SEND_TIMEOUT or on NON_ACK) before failing
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        if self.mock_communication:
            max_retries = 0

        msg.down_id = self._current_msg_id
        self._current_msg_id = (self._current_msg_id + 1) % 256
        serialized = msg.serialize().tobytes()
        for i in range(max_retries):
            # print(serialized)
            self._serial_port.write(serialized)
            time_sent = int(round(time.time() * 1000))
            while self._serial_port.in_waiting < UP_MESSAGE_SIZE:
                if int(round(time.time() * 1000)) - time_sent > SERIAL_SEND_TIMEOUT:
                    break  # waiting for ack
            if self._serial_port.in_waiting >= UP_MESSAGE_SIZE:
                packed = self._serial_port.read(UP_MESSAGE_SIZE)
                up_msg = sMessageUp()
                up_msg.deserialize(packed)
                if up_msg.type == eTypeUp.ACK_UP:
                    return 0  # success
                else:
                    self._mailbox.append(up_msg)  # if it is not an ACK or a NONACK, store it to deliver later
        return -1  # failure

    def check_message(self):
        """
        Check if there is any incoming message on the Serial (defined during the instantiation of the class)
        and returns the oldest message.
        :return: The oldest message non read
        :rtype: sMessageUp
        """
        if self.mock_communication:
            return None

        if self._serial_port.in_waiting >= UP_MESSAGE_SIZE:
            packed = self._serial_port.read(UP_MESSAGE_SIZE)
            up_msg = sMessageUp()
            up_msg.deserialize(packed)
            self._mailbox.append(up_msg)
        if len(self._mailbox) > 0:
            return self._mailbox.popleft()
        return None

