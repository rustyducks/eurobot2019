from communication.message_definition import *
import robot
import time

r = robot.Robot(0)
# r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: print(
#     "X : {}, Y : {}, Theta : {}\t(dx : {}, dy : {}, dt : {}, old report id : {}, new report id : {})".format(
#        r.locomotion.x, r.locomotion.y, r.locomotion.theta, x, y, t, o, n)))
r.communication.register_callback(eTypeUp.ODOM_REPORT, lambda o, n, x, y, t: r.ivy.send_robot_position())
r.communication.register_callback(eTypeUp.HMI_STATE, lambda cord, b1, b2, lr, lg, lb: print("c: {}, b1: {}, b2: {}".format(
    r.io.cord_state, r.io.button1_state ,r.io.button2_state)))
# r.communication.register_callback(eTypeUp.SENSOR_VALUE, lambda i, v: print("Sensor ID : {}, Sensor Value : {}".format(i, v)))

last_behavior_time = time.time()
while 1:
    time.sleep(0.01)
    r.communication.check_message()
    r.locomotion.locomotion_loop(obstacle_detection=True)
    if time.time() - last_behavior_time >= 1:
        r.behavior.loop()
        last_behavior_time = time.time()
    # print([l.distance for l in r.io.lidar_points])
    # print(r.io.is_obstacle_in_cone(0, 20, 200))
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
