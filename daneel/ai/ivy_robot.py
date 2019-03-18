from ivy.std_api import *



IVY_APP_NAME = "AI_Robot"

NEW_OBSTACLE_REGEXP = "New Obstacle {}"  # New Obstacle id : 3 type : POLYGON points : 1500,350;1500,650;1000,650;1000,350
GO_TO_ORIENT_REGEXP = "Go to orient (.*)"
GO_TO_REGEXP = "Go to linear (.*)"
NEW_TRAJECTORY_REGEXP = "New trajectory {}"
UPDATE_ROBOT_POSITION_REGEXP = "Update robot pose {}"
HIGHLIGHT_POINT_REGEXP = "Highlight point {}"  # Highlight point id;x;y
HIGHLIGHT_ANGLE_REGEXP = "Highlight angle {}"  # Highlight angle id;theta
CUSTOM_ACTION_REGEXP = "Custom action (.*)"  # Custom action 5
SPEED_DIRECTION_REGEXP = "Direction (.*)"  # eg : Direction 1,1,1  or 1,0,-1 (vertical, horiztonal, orientation)





class Ivy:
    def __init__(self, robot, bus):
        self.robot = robot
        IvyInit(IVY_APP_NAME, IVY_APP_NAME + "online", 0, self.on_new_connexion, lambda agent, event: None)
        IvyStart(bus)

    def on_new_connexion(self, agent, event):
        if agent.agent_name == "Pygargue":
            self.send_robot_position()
            for obstacle in self.robot.map.lidar_static_obstacles_bb:
                IvySendMsg(NEW_OBSTACLE_REGEXP.format(obstacle.serialize()))
                print(obstacle.serialize())

    def register_callback(self, regexp, callback):
        IvyBindMsg(callback, regexp)

    def send_robot_position(self):
        IvySendMsg(UPDATE_ROBOT_POSITION_REGEXP.format(str(self.robot.locomotion.x) + ";" + str(self.robot.locomotion.y)
                                                       + ";" + str(self.robot.locomotion.theta)))

    def highlight_point(self, ident, x, y):
        IvySendMsg(HIGHLIGHT_POINT_REGEXP.format(str(ident) + ';' + str(x) + ';' + str(y)))

    def highlight_robot_angle(self, ident, theta):
        IvySendMsg(HIGHLIGHT_ANGLE_REGEXP.format(str(ident) + ";" + str(theta)))

    def send_trajectory(self):
        traj = ""
        for point in self.robot.locomotion.position_control.trajectory:
            pt = point.point
            traj += str(pt.x) + "," + str(pt.y)
            traj += ";"
        IvySendMsg(NEW_TRAJECTORY_REGEXP.format(traj[:-1]))