from communication import *
from communication.message_definition import *
import robot
import time
import ivy_robot

i = 0
speedCommands = [(2, 0, 1), (0, 2, 1), (-2, 0, 1), (0, -2, 1)]
r = robot.Robot()
r.ivy = ivy_robot.Ivy(r, "192.168.1.19:2010")
r.locomotion.go_to_orient(1500, 1000, 0)
r.communication.register_callback(eTypeUp.ODOM_REPORT, r.locomotion.handle_new_odometry_report)
r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: print(
    "X : {}, Y : {}, Theta : {}\t(dx : {}, dy : {}, dt : {}, old report id : {}, new report id : {})".format(
        r.locomotion.x, r.locomotion.y, r.locomotion.theta, x, y, t, o, n)))
r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: r.ivy.send_robot_position())
while 1:
    time.sleep(0.01)
    i = (i+1) % len(speedCommands)
    #print('Sending')
    r.communication.check_message()
    r.locomotion.position_control_loop()
    # if msg is not None:
    #     print(msg.type)
    #     if msg.type == eTypeUp.HMI_STATE:
    #         hmi_state = msg.data.hmi_state
    #         print("HMI State : {}".format(hmi_state))
    #     if msg.type == eTypeUp.ODOM_REPORT:
    #         print("Old report : {}, New report : {}, dx : {}, dy : {}, dtheta : {}".format(msg.data.previous_report_id,
    #                                                                                        msg.data.new_report_id,
    #                                                                                        msg.data.dx, msg.data.dy,
    #                                                                                        msg.data.dtheta))
