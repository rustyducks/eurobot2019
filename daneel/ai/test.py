from communication.communication import *
from communication.message_definition import *
import time

comm = Communication("/dev/ttyS0")
comm.mock_communication = False
i = 0
speedCommands = [(2, 0, 1), (0, 2, 1), (-2, 0, 1), (0, -2, 1)]
while 1:
    msg = sMessageDown()
    msg.type = eTypeDown.SPEED_COMMAND
    msg.data = sSpeedCommand()
    msg.data.vx = speedCommands[i][0]
    msg.data.vy = speedCommands[i][1]
    msg.data.vtheta = speedCommands[i][2]
    comm.send_message(msg)
    time.sleep(1)
    i = (i+1) % len(speedCommands)
    print('Sending')
    msg = comm.check_message()
    if msg is not None:
        print(msg.type)
        if msg.type == eTypeUp.HMI_STATE:
            hmi_state = msg.data.hmi_state
            print("HMI State : {}".format(hmi_state))
        if msg.type == eTypeUp.ODOM_REPORT:
            print("Old report : {}, New report : {}, dx : {}, dy : {}, dtheta : {}".format(msg.data.previous_report_id,
                                                                                           msg.data.new_report_id,
                                                                                           msg.data.dx, msg.data.dy,
                                                                                           msg.data.dtheta))
