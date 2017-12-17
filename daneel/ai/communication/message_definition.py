from enum import Enum
import bitstring


UP_MESSAGE_SIZE = 11  # maximum size of a up message (teensy -> raspi) in bytes
DOWN_MESSAGE_SIZE = 9  # maximum size of a down message (raspi -> teensy) in bytes

# ======= Up (Prop -> raspi) message declaration ======== #


class eTypeUp(Enum):
    ACK_UP = 0
    ODOM_REPORT = 1
    HMI_STATE = 2
    ACTUATOR_STATE = 3


class sAckDown:
    def __init__(self):
        self.ack_down_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8', self.ack_down_id)


class sOdomReport:
    def __init__(self):
        self.previous_report_id = None  # uint:8
        self.new_report_id = None  # uint:8
        self.dx = None  # uint:16
        self.dy = None  # uint:16
        self.dtheta = None  # uint:16

    def serialize(self):
        return bitstring.pack('uint:8, uint:8, uintle:16, uintle:16, uintle:16',
                              self.previous_report_id, self.new_report_id,
                              self.dx, self.dy, self.dtheta)


class sHMIState:
    def __init__(self):
        self.hmi_state = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8', self.hmi_state)


class sActuatorState:
    def __init__(self):
        self.actuator_id = None  # uint:8
        self.actuator_value = None  # uint:16

    def serialize(self):
        return bitstring.pack('uint:8, uintle:16', self.actuator_id,
                              self.actuator_value)


class sMessageUp:
    """
    Class defining the up (teensy -> raspi) messages
    :type type: eTypeUp
    :type up_id: int
    :type checksum: int
    :type data: sAckDown|sActuatorState|sHMIState|sOdomReport
    """
    def __init__(self):
        self.up_id = None
        self.type = None
        self.checksum = None
        self.data = None

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()
            self.checksum = 0
            for octet in ser2.tobytes():
                self.checksum += octet
            self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8', self.up_id, self.type.value, self.checksum)
        serialized_msg = ser + ser2
        pad = bitstring.pack('pad:{}'.format((UP_MESSAGE_SIZE - len(serialized_msg.tobytes())) * 8))
        return serialized_msg + pad


# ====== End up message declaration
# ====== Down (raspi -> prop) message declaration #


class eTypeDown(Enum):
    ACK_DOWN = 0
    ACK_ODOM_REPORT = 1
    SPEED_COMMAND = 2
    ACTUATOR_COMMAND = 3
    HMI_COMMAND = 4


class sAckUp:
    def __init__(self):
        self.ack_up_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8', self.ack_up_id)


class sAckOdomReport:
    def __init__(self):
        self.ack_up_id = None  # uint:8
        self.ack_odom_report_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8, uint:8', self.ack_up_id, self.ack_odom_report_id)


class sSpeedCommand:
    def __init__(self):
        self.vx = None  # uint:16
        self.vy = None  # uint:16
        self.vtheta = None  # uint:16

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16', self.vx, self.vy,
                              self.vtheta)


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


class sMessageDown:
    """
    Class defining the down (raspi -> teensy) messages
    :type type: eTypeDown
    :type down_id: int
    :type checksum: int
    :type data: sAckOdomReport|sAckUp|sActuatorCommand|sSpeedCommand|sHMICommand
    """
    def __init__(self):
        self.down_id = 0  # :8
        self.type = None  # :8
        self.checksum = 0  # :8
        self.data = None

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()

            self.checksum = 0

            for octet in ser2.tobytes():
                self.checksum += octet

        self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8', self.down_id, self.type.value, self.checksum)
        serialized_msg = ser + ser2
        pad = bitstring.pack('pad:{}'.format((UP_MESSAGE_SIZE - len(serialized_msg.tobytes())) * 8))
        return serialized_msg + pad


# ====== End down message declaration
