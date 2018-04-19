"""
Modified from : https://github.com/Xevel/NXV11
Under Apache 2.0 License
"""

import time
import math
import serial

lidar_points = [None]*360  # type: list[LidarPoint]
packet_per_cyle = int(359/4)  # In order to flush the input on each rotation

class LidarPoint:
    def __init__(self, azimut=0, distance=0, quality=0, valid=False, warning=True, updTour=0, point = None):
        if point is not None:
            self.azimut = point.azimut  # type: int
            self.distance = point.distance  # type: int
            self.quality = point.quality  # type: int
            self.valid = point.valid  # type: bool
            self.warning = point.warning  # type: bool
            self.updTour = point.updTour  # type: int
        else:
            self.azimut = azimut  # type: int
            self.distance = distance  # type: int
            self.quality = quality  # type: int
            self.valid = valid  # type: bool
            self.warning = warning  # type: bool
            self.updTour = updTour  # type: int

    def get_cartesian_coord(self):
        return (self.distance * math.cos(math.radians(self.azimut)),
                self.distance * math.sin(math.radians(self.azimut)))


def read_v_2_4(lidar_serial):
    global lidar_points
    init_level = 0
    index = 0
    cycle = 0
    while True:
        try:
            time.sleep(0.00001)  # do not hog the processor power
            if init_level == 0:
                b = lidar_serial.read(1)
                # start byte
                if b == bytes([0xFA]):
                    init_level = 1
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = lidar_serial.read(1)
                if bytes([0xA0]) <= b <= bytes([0xF9]):
                    index = int.from_bytes(b, byteorder='big') - 0xA0
                    init_level = 2
                elif b != bytes([0xFA]):
                    init_level = 0
            elif init_level == 2:
                # speed
                b_speed = [b for b in lidar_serial.read(2)]

                # data
                b_data0 = [b for b in lidar_serial.read(4)]
                b_data1 = [b for b in lidar_serial.read(4)]
                b_data2 = [b for b in lidar_serial.read(4)]
                b_data3 = [b for b in lidar_serial.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [0xFA, index + 0xA0] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [b for b in lidar_serial.read(2)]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    # speed_rpm = compute_speed(b_speed)
                    # gui_update_speed(speed_rpm)

                    # motor_control(speed_rpm)

                    lidar_points[index * 4 + 0] = new_lidar_point(index * 4 + 0, b_data0)
                    lidar_points[index * 4 + 1] = new_lidar_point(index * 4 + 1, b_data1)
                    lidar_points[index * 4 + 2] = new_lidar_point(index * 4 + 2, b_data2)
                    lidar_points[index * 4 + 3] = new_lidar_point(index * 4 + 3, b_data3)

                    if index == packet_per_cyle:
                        cycle = (cycle + 1) % 2
                        if cycle == 0:
                            lidar_serial.flushInput()

                else:
                    pass
                    # the checksum does not match, something went wrong...
                    # nb_errors += 1
                    # label_errors.text = "errors: " + str(nb_errors)
                    #
                    # # display the samples in an error state
                    # update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 3, [0, 0x80, 0, 0])

                init_level = 0  # reset and wait for the next packet

            else:  # default, should never happen...
                init_level = 0
        except Exception as err:
            print(err)


def new_lidar_point(angle, data):
    x = data[0]
    x1 = data[1]
    x2 = data[2]
    x3 = data[3]
    dist_mm = x | ((x1 & 0x3f) << 8)  # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8)  # quality is on 16 bits
    return LidarPoint(angle, dist_mm, quality, not (x1 & 0x80),  bool(x1 & 0x40), 0)


def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived in.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF  # truncate to 15 bits
    return int(checksum)

