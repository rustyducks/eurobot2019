from communication.communication import *
from communication.message_definition import *
import time

comm = Communication("/dev/ttyS0")
comm.mock_communication = False
i = 0
hmiCommands = [(0, 0, 0), (0, 0, 1), (0, 1, 1), (1, 1, 1)]
while 1:
    msg = sMessageDown()
    msg.type = eTypeDown.HMI_COMMAND
    msg.data = sHMICommand()
    msg.data.hmi_command = hmiCommands[i][0] << 7 | hmiCommands[i][1] << 6 | hmiCommands[i][2] << 5
    comm.send_message(msg)
    time.sleep(1)
    i = (i+1) % len(hmiCommands)
    print('Sending')
    msg = comm.check_message()
    if msg is not None:
        print(msg.type)
        if msg.type == eTypeUp.HMI_STATE:
            hmi_state = msg.data.hmi_state
            print("HMI State : {}".format(hmi_state))
        if msg.type == eTypeUp.ODOM_REPORT:
            print("Old report : {}, New report : {}, dx : {}, dy : {}, dtheta : {}".format(
                msg.data.previous_report_id, msg.data.new_report_id, msg.data.dx, msg.data.dy, msg.data.dtheta
            ))
