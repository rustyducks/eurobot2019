from communication.message_definition import *
import robot
import time

r = robot.Robot(2)
# r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: print(
#     "X : {}, Y : {}, Theta : {}\t(dx : {}, dy : {}, dt : {}, old report id : {}, new report id : {})".format(
#        r.locomotion.x, r.locomotion.y, r.locomotion.theta, x, y, t, o, n)))
r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: r.ivy.send_robot_position())
while 1:
    time.sleep(0.01)
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
