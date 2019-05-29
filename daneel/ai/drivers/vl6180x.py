"""
driver for the VL6180X, tof range and ambiant light sensor

NOTE : this driver only support single-shot ranging (no continuous, no ambiant light)

See "AN4545 - VL6180X basic ranging application note" (https://www.st.com/content/ccc/resource/technical/document/application_note/d5/bb/ec/94/7d/1e/40/a0/DM00122600.pdf/files/DM00122600.pdf/jcr:content/translations/en.DM00122600.pdf)

Created by Fabien-B, 2019-04-26.
"""

import smbus2
from enum import Enum

IDENTIFICATION_MODEL_ID = 0x0000
IDENTIFICATION_MODEL_REV_MAJOR = 0x0001
IDENTIFICATION_MODEL_REV_MINOR = 0x0002
IDENTIFICATION_MODULE_REV_MAJOR = 0x0003
IDENTIFICATION_MODULE_REV_MINOR = 0x0004
IDENTIFICATION_DATE_HI = 0x0006
IDENTIFICATION_DATE_LO = 0x0007
IDENTIFICATION_TIME = 0x0008
SYSTEM_MODE_GPIO0 = 0x0010
SYSTEM_MODE_GPIO1 = 0x0011
SYSTEM_HISTORY_CTRL = 0x0012
SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0014
SYSTEM_INTERRUPT_CLEAR = 0x0015
SYSTEM_FRESH_OUT_OF_RESET = 0x0016
SYSTEM_GROUPED_PARAMETER_HOLD = 0x0017
SYSRANGE_START = 0x0018
SYSRANGE_THRESH_HIGH = 0x0019
SYSRANGE_THRESH_LOW = 0x001A
SYSRANGE_INTERMEASUREMENT_PERIOD = 0x001B
SYSRANGE_MAX_CONVERGENCE_TIME = 0x001C
SYSRANGE_CROSSTALK_COMPENSATION_RATE = 0x001E
SYSRANGE_CROSSTALK_VALID_HEIGHT = 0x0021
SYSRANGE_EARLY_CONVERGENCE_ESTIMATE = 0x0022
SYSRANGE_PART_TO_PART_RANGE_OFFSET = 0x0024
SYSRANGE_RANGE_IGNORE_VALID_HEIGHT = 0x0025
SYSRANGE_RANGE_IGNORE_THRESHOLD = 0x0026
SYSRANGE_MAX_AMBIENT_LEVEL_MULT = 0x002C
SYSRANGE_RANGE_CHECK_ENABLES = 0x002D
SYSRANGE_VHV_RECALIBRATE = 0x002E
SYSRANGE_VHV_REPEAT_RATE = 0x0031
SYSALS_START = 0x0038
SYSALS_THRESH_HIGH = 0x003A
SYSALS_THRESH_LOW = 0x003C
SYSALS_INTERMEASUREMENT_PERIOD = 0x003E
SYSALS_ANALOGUE_GAIN = 0x003F
SYSALS_INTEGRATION_PERIOD = 0x0040
RESULT_RANGE_STATUS = 0x004D
RESULT_ALS_STATUS = 0x004E
RESULT_INTERRUPT_STATUS_GPIO = 0x004F
RESULT_ALS_VAL = 0x0050
RESULT_HISTORY_BUFFER_x = 0x0052
RESULT_RANGE_VAL = 0x0062
RESULT_RANGE_RAW = 0x0064
RESULT_RANGE_RETURN_RATE = 0x0066
RESULT_RANGE_REFERENCE_RATE = 0x0068
RESULT_RANGE_RETURN_SIGNAL_COUNT = 0x006C
RESULT_RANGE_REFERENCE_SIGNAL_COUNT = 0x0070
RESULT_RANGE_RETURN_AMB_COUNT = 0x0074
RESULT_RANGE_REFERENCE_AMB_COUNT = 0x0078
RESULT_RANGE_RETURN_CONV_TIME = 0x007C
RESULT_RANGE_REFERENCE_CONV_TIME = 0x0080
READOUT_AVERAGING_SAMPLE_PERIOD = 0x010A
FIRMWARE_BOOTUP = 0x0119
FIRMWARE_RESULT_SCALER = 0x0120
I2C_SLAVE_DEVICE_ADDRESS = 0x0212
INTERLEAVED_MODE_ENABLE = 0x02A3


class RangeError(Enum):
    NO_ERROR = 0b000
    VCSEL_CONTINUITY_TEST = 0b0001
    VCSEL_WATCHDOG_TEST = 0b0010
    VCSEL_WATCHDOG = 0b0011
    PLL1_LOCK = 0b0100
    PLL2_LOCK = 0b0101
    EARLY_CONVERGENCE_ESTIMATE = 0b0110
    MAX_CONVERGENCE = 0b0111
    NO_TARGET_IGNORE = 0b1000
    MAX_SIGNAL_TO_NOISE_RATIO = 0b1011
    RAW_RANGING_ALGO_UNDERFLOW = 0b1100
    RAW_RANGING_ALGO_OVERFLOW = 0b1101
    RANGING_ALGO_UNDERFLOW = 0b1110
    RANGING_ALGO_OVERFLOW = 0b1111


BUS = smbus2.SMBus(1)

class VL6180X():
    def __init__(self, addr=0x29):
        """
        addr: 7 bits address of the device. 0X29 by default for the VL6180X.
        """
        self.address = addr
    
    def write_bytes(self, register, data=[]):
        """
        register: 16 bits register address
        data: optionnal data to write to this address
        """
        bs = [(register&0XFF00)>>8, register&0X00FF]
        bs.extend(data)
        BUS.i2c_rdwr(smbus2.i2c_msg.write(self.address, bs))
    
    def write(self, register, data=None):
        """
        register: 16 bits register address
        data: optionnal data to write to this address
        """
        l=[]
        if data is not None:
            l.append(data)
        self.write_bytes(register, l)
    
    def read_byte(self):
        return BUS.read_byte(self.address)
    
    def read_from(self, register, nb=1):
        """
        Read byte(s) from _register_.
        """
        self.write(register)
        if nb==1:
            return self.read_byte()
        else:
            res = []
            for _ in range(nb):
                res.append(self.read_byte())
            return res
    
    def change_address(self, new_address):
        self.write(I2C_SLAVE_DEVICE_ADDRESS, new_address)
        self.address = new_address
    
    def load_sr_settings(self):
        """
        Loads required settings at startup.
        See AN4545 - VL6180X basic ranging application note
        """
        # Mandatory : private registers
        self.write(0x0207, 0x01)
        self.write(0x0208, 0x01)
        self.write(0x0096, 0x00)
        self.write(0x0097, 0xfd)
        self.write(0x00e3, 0x01)
        self.write(0x00e4, 0x03)
        self.write(0x00e5, 0x02)
        self.write(0x00e6, 0x01)
        self.write(0x00e7, 0x03)
        self.write(0x00f5, 0x02)
        self.write(0x00d9, 0x05)
        self.write(0x00db, 0xce)
        self.write(0x00dc, 0x03)
        self.write(0x00dd, 0xf8)
        self.write(0x009f, 0x00)
        self.write(0x00a3, 0x3c)
        self.write(0x00b7, 0x00)
        self.write(0x00bb, 0x3c)
        self.write(0x00b2, 0x09)
        self.write(0x00ca, 0x09)
        self.write(0x0198, 0x01)
        self.write(0x01b0, 0x17)
        self.write(0x01ad, 0x00)
        self.write(0x00ff, 0x05)
        self.write(0x0100, 0x05)
        self.write(0x0199, 0x05)
        self.write(0x01a6, 0x1b)
        self.write(0x01ac, 0x3e)
        self.write(0x01a7, 0x1f)
        self.write(0x0030, 0x00)
        
        # Recommended : Public registers - See data sheet for more detail
        self.write(0x0011, 0x10) # Enables polling for ‘New Sample ready’ when measurement completes
        self.write(0x010a, 0x30) # Set the averaging sample period (compromise between lower noise and increased execution time)
        self.write(0x003f, 0x46) # Sets the light and dark gain (upper nibble). Dark gain should not be changed.
        self.write(0x0031, 0xFF) # sets the # of range measurements after which auto calibration of system is performed
        self.write(0x0041, 0x63) # Set ALS integration time to 100ms
        self.write(0x002e, 0x01) # perform a single temperature calibration of the ranging sensor Optional: Public registers - See data sheet for more detail
        self.write(0x001b, 0x09) # Set default ranging inter-measurement period to 100ms
        self.write(0x003e, 0x31) # Set default ALS inter-measurement period to 500ms
        self.write(0x0014, 0x24) # Configures interrupt on ‘New Sample Ready threshold event’

    def init(self):
        """
        Initialize device.
        """
        val = self.read_from(SYSTEM_FRESH_OUT_OF_RESET)
        #print(val)
        if not val == 0X01:
            print("init ERROR, SYSTEM_FRESH_OUT_OF_RESET should be equal to 0X01. Maybe the device is already initialized ?")
        self.load_sr_settings()
        self.write(SYSTEM_FRESH_OUT_OF_RESET, 0X00)
    
    def get_ready(self, new_addr):
        have_good_address = False
        try:
            self.write(IDENTIFICATION_MODEL_ID)
            have_good_address = True
        except OSError:
            print("OSError #1 !")
            have_good_address = False
        if not have_good_address:
            self.address = new_addr
            try:
                self.write(IDENTIFICATION_MODEL_ID)
                have_good_address = True
            except OSError:
                print("OSError #2 !")
                have_good_address = False
        if have_good_address:
            m_id = self.read_byte()
            if not m_id == 0xB4:
                print("model id is different than 0XB4: {}".format(hex(m_id)))
            self.init()
            self.change_address(new_addr)
            return 0
        else:
            return -1
        

    def do_single_shot(self):
        """
        Performs a single-shot ranging.
        return measured distance in mm.
        """
        # Check device is ready to start a range measurement.
        val = self.read_from(RESULT_RANGE_STATUS)
        if not val & 0X01:
            print("device {} not ready ! Try it anyway.".format(self.address))
        # Start a range measurement
        self.write(SYSRANGE_START, 0X01)
        # wait for range measurement to complete
        while True:
            val = self.read_from(RESULT_INTERRUPT_STATUS_GPIO)
            if val & 0b100:
                break
        # get range value (in mm)
        distance = self.read_from(RESULT_RANGE_VAL)
        # clear interrupt status
        self.write(SYSTEM_INTERRUPT_CLEAR, 0X07)
        
        status = RangeError(self.read_from(RESULT_RANGE_STATUS)>>4)

        return distance, status


