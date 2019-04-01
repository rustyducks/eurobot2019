from enum import Enum
import bitstring

UP_MESSAGE_SIZE = 10  # maximum size of a up message (teensy -> raspi) in bytes
UP_HEADER_SIZE = 4  # size of the header (all except the data) of an up message
DOWN_MESSAGE_SIZE = 16  # maximum size of a down message (raspi -> teensy) in bytes
DOWN_HEADER_SIZE = 4  # size of the header (all except the data) of a down message

# Data converters (from and to what is sent over the wire and what is used as data).
LINEAR_POSITION_TO_MSG_FACTOR = 4
LINEAR_POSITION_TO_MSG_ADDER = 8192
RADIAN_TO_MSG_FACTOR = 10430.378350470453
RADIAN_TO_MSG_ADDER = 3.14159265358979323846
LINEAR_SPEED_TO_MSG_FACTOR = 32.768
LINEAR_SPEED_TO_MSG_ADDER = 1000
ANGULAR_SPEED_TO_MSG_FACTOR = 2607.5945876176133
ANGULAR_SPEED_TO_MSG_ADDER = 4 * 3.14159265358979323846


class DeserializationException(Exception):
    """
    Raised on deserialization error (when trying to read what is sent over the wire into usable data)
    """

    def __init__(self, message):
        """
        Ctor of the deserialization exception.

        :param message: Message that will be thrown with the exception (and may be printed on the user screen)
        :type message: str
        """
        super().__init__(message)


# ======= Up (Prop -> raspi) message declaration ======== #

class eTypeUp(Enum):
    """
    All the message types that can be sent from the base (Teensy) to the ai.
    Must be the same as base/code/communication/Communication.h:eUpMessageType
    """
    ACK_DOWN = 0
    ODOM_REPORT = 1
    HMI_STATE = 2
    SENSOR_VALUE = 3
    SPEED_REPORT = 4


class sAckDown:
    """
    Payload of an up message (from base to ai) acknowledging a down message (from ai to base).
    """

    def __init__(self):
        """
        Ctor
        """
        self.ack_down_id = None  # uint:8

    def deserialize(self, bytes_packed):
        """
        Fills the id of the acknowledged message with no alteration from what is received from the wire.

        :param bytes_packed: The bytes received from serial.
        :type bytes_packed: list[bytes]
        """
        s = bitstring.BitStream(bytes_packed)
        self.ack_down_id, = s.unpack('uint:8')

    def serialize(self):
        """
        Not used.

        :return:
        :rtype:
        """
        return bitstring.pack('uint:8', self.ack_down_id)


class sOdomReport:
    def __init__(self):
        self._x = None  # uint:16
        self._y = None  # uint:16
        self._theta = None  # uint:16

    @property
    def x(self):
        return self._x / LINEAR_POSITION_TO_MSG_FACTOR - LINEAR_POSITION_TO_MSG_ADDER

    @x.setter
    def x(self, x):
        self._x = (x + LINEAR_POSITION_TO_MSG_ADDER) * LINEAR_POSITION_TO_MSG_FACTOR

    @property
    def y(self):
        return self._y / LINEAR_POSITION_TO_MSG_FACTOR - LINEAR_POSITION_TO_MSG_ADDER

    @y.setter
    def y(self, y):
        self._y = (y + LINEAR_POSITION_TO_MSG_ADDER) * LINEAR_POSITION_TO_MSG_FACTOR

    @property
    def theta(self):
        return self._theta / RADIAN_TO_MSG_FACTOR - RADIAN_TO_MSG_ADDER

    @theta.setter
    def theta(self, theta):
        self._theta = (theta + RADIAN_TO_MSG_ADDER) * RADIAN_TO_MSG_FACTOR

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self._x, self._y, self._theta = s.unpack(
            'uintle:16, uintle:16, uintle:16')

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16',
                              self._x, self._y, self._theta)


class sHMIState:
    def __init__(self):
        self.hmi_state = None  # uint:8

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self.hmi_state, = s.unpack('uint:8')

    def serialize(self):
        return bitstring.pack('uint:8', self.hmi_state)


class sSpeedReport:
    def __init__(self):
        self._vx = None  #  uint:16
        self._vy = None  #  uint:16
        self._vtheta = None  #  uint:16

    @property
    def vx(self):
        return self._vx / LINEAR_SPEED_TO_MSG_FACTOR - LINEAR_SPEED_TO_MSG_ADDER

    @vx.setter
    def vx(self, value):
        self._vx = round((value + LINEAR_SPEED_TO_MSG_ADDER) * LINEAR_SPEED_TO_MSG_FACTOR)

    @property
    def vy(self):
        return self._vy / LINEAR_SPEED_TO_MSG_FACTOR - LINEAR_SPEED_TO_MSG_ADDER

    @vy.setter
    def vy(self, value):
        self._vy = round((value + LINEAR_SPEED_TO_MSG_ADDER) * LINEAR_SPEED_TO_MSG_FACTOR)

    @property
    def vtheta(self):
        return self._vtheta / ANGULAR_SPEED_TO_MSG_FACTOR - ANGULAR_SPEED_TO_MSG_ADDER

    @vtheta.setter
    def vtheta(self, value):
        self._vtheta = round((value + ANGULAR_SPEED_TO_MSG_ADDER) * ANGULAR_SPEED_TO_MSG_FACTOR)

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self._vx, self._vy, self._vtheta = s.unpack(
            'uintle:16, uintle:16, uintle:16')

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16', self._vx, self._vy, self._vtheta)


class sSensorValue:
    def __init__(self):
        self.sensor_id = None
        self.sensor_value = None

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self.sensor_id, self.sensor_value = s.unpack('uint:8, uintle:16')


class sMessageUp:
    """
    Class defining the up (teensy -> raspi) messages
    :type type: eTypeUp|None
    :type up_id: int|None
    :type checksum: int|None
    :type data: sAckDown|sHMIState|sOdomReport|sSensorValue|sSpeedReport|None
    """

    def __init__(self):
        self.up_id = None
        self.type = None
        self.data_size = None
        self.checksum = None
        self.data = None

    def deserialize_header(self, packed):
        header = bitstring.BitStream(packed[0:UP_HEADER_SIZE])
        try:
            self.up_id, type_value, self.data_size, self.checksum = header.unpack('uint:8, uint:8, uint:8, uint:8')
            self.type = eTypeUp(type_value)
        except ValueError as e:
            raise DeserializationException("Can't deserialize up message header : {}".format(str(e)))

    def deserialize_data(self, packed):
        if self.type == eTypeUp.ACK_DOWN:
            self.data = sAckDown()
        elif self.type == eTypeUp.HMI_STATE:
            self.data = sHMIState()
        elif self.type == eTypeUp.ODOM_REPORT:
            self.data = sOdomReport()
        elif self.type == eTypeUp.SENSOR_VALUE:
            self.data = sSensorValue()
        elif self.type == eTypeUp.SPEED_REPORT:
            self.data = sSpeedReport()
        try:
            self.data.deserialize(packed[0:self.data_size])
        except ValueError as e:
            raise DeserializationException("Can't deserialize up message payload : {}".format(str(e)))

    def deserialize(self, packed):
        self.deserialize_header(packed)
        self.deserialize(packed[UP_HEADER_SIZE:])

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()
            self.data_size = len(ser2.tobytes())
            self.checksum = 0
            for octet in ser2.tobytes():
                self.checksum ^= octet
            self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8, uint:8',
                             self.up_id, self.type.value, self.data_size, self.checksum)
        serialized_msg = ser + ser2
        return serialized_msg


# ====== End up message declaration
# ====== Down (raspi -> prop) message declaration #


class eTypeDown(Enum):
    ACK_UP = 0
    SPEED_COMMAND = 1
    ACTUATOR_COMMAND = 2
    HMI_COMMAND = 3
    RESET = 4
    REPOSITIONING = 5
    PID_TUNING = 6
    SENSOR_COMMAND = 7


class sAckUp:
    def __init__(self):
        self.ack_up_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8', self.ack_up_id)


class sSpeedCommand:
    def __init__(self):
        self._vx = None  # uint:16
        self._vy = None  # uint:16
        self._vtheta = None  # uint:16

    @property
    def vx(self):
        return self._vx / LINEAR_SPEED_TO_MSG_FACTOR - LINEAR_SPEED_TO_MSG_ADDER

    @vx.setter
    def vx(self, vx):
        self._vx = round((vx + LINEAR_SPEED_TO_MSG_ADDER) * LINEAR_SPEED_TO_MSG_FACTOR)

    @property
    def vy(self):
        return self._vy / LINEAR_SPEED_TO_MSG_FACTOR - LINEAR_SPEED_TO_MSG_ADDER

    @vy.setter
    def vy(self, vy):
        self._vy = round((vy + LINEAR_SPEED_TO_MSG_ADDER) * LINEAR_SPEED_TO_MSG_FACTOR)

    @property
    def vtheta(self):
        return self._vtheta / ANGULAR_SPEED_TO_MSG_FACTOR - ANGULAR_SPEED_TO_MSG_ADDER

    @vtheta.setter
    def vtheta(self, vtheta):
        self._vtheta = round((vtheta + ANGULAR_SPEED_TO_MSG_ADDER) * ANGULAR_SPEED_TO_MSG_FACTOR)

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16', self._vx, self._vy,
                              self._vtheta)


class sActuatorCommand:
    def __init__(self):
        self.actuator_id = None  # uint:8
        self.actuator_command = None  # uint:16

    def serialize(self):
        return bitstring.pack('uint:8, uintle:16', self.actuator_id, self.actuator_command)


class sHMICommand:
    def __init__(self):
        self.hmi_command = None

    def serialize(self):
        return bitstring.pack('uint:8', self.hmi_command)


class sRepositioning:
    def __init__(self):
        self._x_repositioning = None
        self._y_repositioning = None
        self._theta_repositioning = None

    @property
    def x_repositioning(self):
        return self._x_repositioning / LINEAR_POSITION_TO_MSG_FACTOR - LINEAR_POSITION_TO_MSG_ADDER

    @x_repositioning.setter
    def x_repositioning(self, value):
        self._x_repositioning = round((value + LINEAR_POSITION_TO_MSG_ADDER) * LINEAR_POSITION_TO_MSG_FACTOR)

    @property
    def y_repositioning(self):
        return self._y_repositioning / LINEAR_POSITION_TO_MSG_FACTOR - LINEAR_POSITION_TO_MSG_ADDER

    @y_repositioning.setter
    def y_repositioning(self, value):
        self._y_repositioning = round((value + LINEAR_POSITION_TO_MSG_ADDER) * LINEAR_POSITION_TO_MSG_FACTOR)

    @property
    def theta_repositioning(self):
        return self._theta_repositioning / RADIAN_TO_MSG_FACTOR - RADIAN_TO_MSG_ADDER

    @theta_repositioning.setter
    def theta_repositioning(self, value):
        self._theta_repositioning = round((value + RADIAN_TO_MSG_ADDER) * RADIAN_TO_MSG_FACTOR)

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16',
                              self._x_repositioning, self._y_repositioning, self._theta_repositioning)


class sPIDTuning:
    def __init__(self):
        self._kp_linear = None
        self._ki_linear = None
        self._kd_linear = None
        self._kp_angular = None
        self._ki_angular = None
        self._kd_angular = None

    @property
    def kp_linear(self):
        return self._kp_linear / 1000

    @kp_linear.setter
    def kp_linear(self, value):
        self._kp_linear = value * 1000

    @property
    def ki_linear(self):
        return self._ki_linear / 1000

    @ki_linear.setter
    def ki_linear(self, value):
        self._ki_linear = value * 1000

    @property
    def kd_linear(self):
        return self._kd_linear / 1000

    @kd_linear.setter
    def kd_linear(self, value):
        self._kd_linear = value * 1000

    @property
    def kp_angular(self):
        return self._kp_angular / 1000

    @kp_angular.setter
    def kp_angular(self, value):
        self._kp_angular = value * 1000

    @property
    def ki_angular(self):
        return self._ki_angular / 1000

    @ki_angular.setter
    def ki_angular(self, value):
        self._ki_angular = value * 1000

    @property
    def kd_angular(self):
        return self._kd_angular / 1000

    @kd_angular.setter
    def kd_angular(self, value):
        self._kd_angular = value * 1000

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16, uintle:16, uintle:16, uintle:16',
                              self._kp_linear, self._ki_linear, self._kd_linear, self._kp_angular, self._ki_angular,
                              self._kd_angular)


class sSensorCommand:
    def __init__(self):
        self.sensor_id = None
        self.sensor_state = None

    def serialize(self):
        return bitstring.pack('uint:8, uint:8', self.sensor_id, self.sensor_state)


class sMessageDown:
    """
    Class defining the down (raspi -> teensy) messages
    :type type: eTypeDown|None
    :type down_id: int
    :type checksum: int
    :type data: sAckUp|sActuatorCommand|sSpeedCommand|sHMICommand|sRepositioning|None
    """

    def __init__(self):
        self.down_id = 0  # :8
        self.type = None  # :8
        self.data_size = 0  # :8
        self.checksum = 0  # :8
        self.data = None

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()
            self.data_size = len(ser2.tobytes())
            self.checksum = 0

            for octet in ser2.tobytes():
                self.checksum ^= octet

        self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8, uint:8', self.down_id, self.type.value, self.data_size,
                             self.checksum)
        serialized_msg = ser + ser2
        return serialized_msg

# ====== End down message declaration
