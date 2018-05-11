from enum import Enum

from smbus2 import SMBus, SMBusWrapper
import time

DEFAULT_ADDRESS = 0x48

ADC_ON = 1 << 2
SINGLE_ENDED = 1 << 7
MAX_OFFSET = 1038


class LineDetector:
    __ADS7828_CONFIG_SD_DIFFERENTIAL = 0b00000000
    __ADS7828_CONFIG_SD_SINGLE = 0b10000000
    __ADS7828_CONFIG_CS_CH0 = 0b00000000
    __ADS7828_CONFIG_CS_CH2 = 0b00010000
    __ADS7828_CONFIG_CS_CH4 = 0b00100000
    __ADS7828_CONFIG_CS_CH6 = 0b00110000
    __ADS7828_CONFIG_CS_CH1 = 0b01000000
    __ADS7828_CONFIG_CS_CH3 = 0b01010000
    __ADS7828_CONFIG_CS_CH5 = 0b01100000
    __ADS7828_CONFIG_CS_CH7 = 0b01110000
    __ADS7828_CONFIG_PD_OFF = 0b00000000
    __ADS7828_CONFIG_PD_REFOFF_ADON = 0b00000100
    __ADS7828_CONFIG_PD_REFON_ADOFF = 0b00001000
    __ADS7828_CONFIG_PD_REFON_ADON = 0b00001100
    class State(Enum):
        IDLE = "Idle"
        ON_BLACK = "On Black"
        ON_WHITE = "On White"

    def __init__(self, address=DEFAULT_ADDRESS):
        #self._ch_offsets = [0, 63, 722, 464, 30, 537, 489, 1038]
        self._ch_offsets = [-676, -128, -78, -25, 34, 254, 66, -32]
        self.states = [self.State.IDLE] * 8
        self.address = address
        self.bus = SMBus(1)

    def _ch(self, ch):
        return self._ch2cfg[ch & 0x7]

    def _ch_offset(self, ch):
        return self._ch_offsets[ch]

    def get_intensity(self, ch):
        config = 0
        config |= self.__ADS7828_CONFIG_SD_SINGLE
        config |= self.__ADS7828_CONFIG_PD_REFOFF_ADON

        if ch == 0:
            config |= self.__ADS7828_CONFIG_CS_CH0
        elif ch == 1:
            config |= self.__ADS7828_CONFIG_CS_CH1
        elif ch == 2:
            config |= self.__ADS7828_CONFIG_CS_CH2
        elif ch == 3:
            config |= self.__ADS7828_CONFIG_CS_CH3
        elif ch == 4:
            config |= self.__ADS7828_CONFIG_CS_CH4
        elif ch == 5:
            config |= self.__ADS7828_CONFIG_CS_CH5
        elif ch == 6:
            config |= self.__ADS7828_CONFIG_CS_CH6
        elif ch == 7:
            config |= self.__ADS7828_CONFIG_CS_CH7

        data = self.bus.read_i2c_block_data(self.address, config, 2)

        return (data[0] << 8) + data[1] + self._ch_offsets[ch]

    def sense(self):
        for i in range(8):
            isity = self.get_intensity(i)
            if self.states[i] == self.State.IDLE:
                if 3700 <= isity <= 4000:
                    self.states[i] = self.State.ON_BLACK
            if self.states[i] == self.State.ON_BLACK:
                if 2800 <= isity <= 2900:
                    self.states[i] = self.State.ON_WHITE

    def reset(self):
        self.states = [self.State.IDLE] * 8

