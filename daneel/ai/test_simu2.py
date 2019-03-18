#!/usr/bin/python3

import sys
from math import atan2
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QIcon, QPixmap, QPen, QColor, QBrush, QPolygonF, QPainter, QImage, QPalette, QKeyEvent, \
    QMouseEvent
from PyQt5.QtCore import QPointF, pyqtSlot, QSize, QRectF, Qt

import math
import sys
import signal
import numpy as np
from collections import namedtuple

import behavior
import robot
from locomotion.utils import Speed

BACKGROUND_COLOR = (25, 25, 25)
RADIUS = 5
DT = 0.1

MAX_LINEAR_ACC = 1. #m.s^-2
MAX_ROT_ACC = 50     #rad.s^-2

TIME_STEP_PER_PRESS = 1

TrajPoint = namedtuple("GoalPoint", ['point', 'speed'])

def center_radians(theta):
    while theta < -math.pi:
        theta += 2*math.pi
    while theta >= math.pi:
        theta -= 2*math.pi
    return theta


class App(QWidget):

    @property
    def planned_trajectory(self):
        return self.robot.locomotion.position_control.trajectory

    @property
    def goal_point(self):
        return self.robot.locomotion.position_control.goal_point


    def __init__(self, r):
        super().__init__()
        self.title = "Pygargue"
        self.table_left = 0
        self.t = 0
        self.table_top = 0
        self.table_width = 1200
        self.table_height = 800
        self.x_press = None
        self.y_press = None
        self.robot = r
        self.robot_speed_command = [0, 0, 0]
        self.current_speed = [0, 0, 0]
        self.past_poses = []
        self.initUI()
 
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.table_left, self.table_top, self.table_width, self.table_height)
        palette = QPalette()
        palette.setColor(QPalette.Background, QColor(*BACKGROUND_COLOR))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        self.pen = QPen(QColor(255,0,0))
        self.pen.setWidth(1)
        self.brush = QBrush(QColor(0,0,0))
        # painter = QPainter(self)
        # painter.setPen(self.pen)
        # painter.setBrush(self.brush)
        # painter.drawPolygon(self.polygon)
        self.show()

    def paintEvent(self, event):
        self.table_width = min(self.geometry().width(), self.geometry().height() * 3/2)
        # The table will keep the 3/2 ratio whatever the window ratio
        self.table_height = min(self.geometry().height(), self.geometry().width() * 2/3)
        painter = QPainter(self)
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setBrush(QBrush(QColor(255,255,255)))
        painter.drawEllipse(QRectF(self.robot.locomotion.x - RADIUS, self.robot.locomotion.y - RADIUS, 2*RADIUS, 2*RADIUS))
        painter.setPen(QPen(QColor(0,0,0)))
        painter.setBrush(QBrush(QColor(0,0,0)))
        painter.drawLine(QPointF(self.robot.locomotion.x, self.robot.locomotion.y), QPointF(self.robot.locomotion.x + math.cos(self.robot.locomotion.theta) * RADIUS, self.robot.locomotion.y + math.sin(self.robot.locomotion.theta) * RADIUS))
        painter.setPen(QPen(QColor(150,0,20)))
        painter.setBrush(QBrush(QColor(150,0,20)))
        for i in range(len(self.planned_trajectory)-1):
            painter.drawLine(QPointF(self.planned_trajectory[i].point.x, self.planned_trajectory[i].point.y), QPointF(self.planned_trajectory[i+1].point.x, self.planned_trajectory[i+1].point.y))
        if self.goal_point is not None:
            painter.setPen(QPen(QColor(150,100,10)))
            painter.setBrush(QBrush(QColor(200,150,10)))
            painter.drawEllipse(QRectF(self.goal_point[0] - 2, self.goal_point[1] - 2, 4, 4))
        if self.robot.locomotion.position_control.ab is not None:
            painter.setPen(QPen(QColor(150, 100, 10)))
            painter.setBrush(QBrush(QColor(200, 150, 10)))
            painter.drawLine(QPointF(0, 0),
                             QPointF(self.robot.locomotion.position_control.ab.x, self.robot.locomotion.position_control.ab.y))
        for p in self.past_poses:
            painter.setPen(QPen(QColor(20,150,10)))
            painter.setBrush(QBrush(QColor(20,150,10)))
            painter.drawEllipse(QRectF(p[0] - 1, p[1] - 1, 2, 2))

    def keyPressEvent(self, event:QKeyEvent):
        #if event.isAutoRepeat():
        #    return
        key = event.key()
        if key == Qt.Key_Space:
            for _ in range(0, TIME_STEP_PER_PRESS):
                self.robot_move()
                self.repaint()

    def robot_move(self):
        #self.planned_trajectory[0] = (self.robot[0], self.robot[1])
        vx, _, vw = self.robot.locomotion.position_control.compute_speed(DT)
        self.time_step(vx, vw)

    def compute_commands(self):
        lookahead_dist = 10
        #Find the path point closest to the robot
        d = 99999
        t_min = 1
        ith_traj_point = 0
        p_min = None
        for i in range(len(self.planned_trajectory)-1):
            ab = np.subtract(self.planned_trajectory[i+1].point, self.planned_trajectory[i].point)
            ar = np.subtract([self.robot.locomotion.current_pose.x, self.robot.locomotion.current_pose.y], self.planned_trajectory[i].point)
            #print(ab)
            t = np.dot(ar, ab) / np.dot(ab, ab)
            p = np.add(self.planned_trajectory[i], np.dot(t, ab))
            pr = np.subtract(self.robot[:2], p)
            dist = np.linalg.norm(pr)
            #print(p, dist)
            if dist < d:
                d = dist
                t_min = t
                ith_traj_point = i
                p_min = p

        #self.goal_point = p_min
        
        # Find the goal point
        lookhead_left = lookahead_dist
        browse_traj_i = 0
        self.planned_trajectory = self.planned_trajectory[ith_traj_point:]
        t_robot = t_min
        #print(lookahead_t, t_min, goal_t)
        path_len = math.hypot(self.planned_trajectory[1][0] - p_min[0],
                                         self.planned_trajectory[1][1] - p_min[1])
        while lookhead_left > path_len:
            # Go to the next segment and recompute it while lookhead_left > 1. If last traj element is reach, extrapolate or clamp to 1 ?
            t_robot = 0
            browse_traj_i += 1
            lookhead_left -= path_len
            path_len = math.hypot(self.planned_trajectory[browse_traj_i + 1][0] - self.planned_trajectory[browse_traj_i][0],
                                             self.planned_trajectory[browse_traj_i + 1][1] - self.planned_trajectory[browse_traj_i][1])
        


        ab = np.subtract(self.planned_trajectory[browse_traj_i+1], self.planned_trajectory[browse_traj_i])
        goal_t = t_robot + lookhead_left / np.linalg.norm(ab)
        goal_point = np.add(self.planned_trajectory[browse_traj_i], np.dot(goal_t, ab))
        #print(lookhead_left, ab, goal_t) 
        self.goal_point = goal_point

        # Goal point in vehicle coord
        goal_point_r = [-(goal_point[0] - self.robot[0]) * math.sin(self.robot[2]) + (goal_point[1] - self.robot[1]) * math.cos(self.robot[2]), 
                        (goal_point[0] - self.robot[0]) * math.cos(self.robot[2]) + (goal_point[1] - self.robot[1]) * math.sin(self.robot[2])]
        
        # Compute the curvature gamma
        gamma = 2 * goal_point_r[0] / (lookahead_dist ** 2)

        vx = 1
        vtheta = vx * gamma

        return self.saturate_velocities(vx, vtheta)

    def saturate_velocities(self, vx, vtheta):
        vx_s = min(self.robot.locomotion.current_speed[0] + MAX_LINEAR_ACC * DT, max(self.robot.locomotion.current_speed[0] - MAX_LINEAR_ACC * DT, vx))
        vtheta_s = min(self.robot.locomotion.current_speed[2] + MAX_ROT_ACC * DT, max(self.robot.locomotion.current_speed[2] - MAX_ROT_ACC * DT, vtheta))
        print("Computed Velocities : {}\tSaturated Velocities : {}".format((vx, vtheta), (vx_s, vtheta_s)))
        return vx_s, vtheta_s

    def time_step(self, vx, vw):
        self.t += DT
        if int(self.t*10) % 10 == 0:
            self.past_poses.append((self.robot.locomotion.current_pose.x, self.robot.locomotion.current_pose.y))
            print("plop")
        new_theta = center_radians(self.robot.locomotion.theta + vw * DT)
        new_x = self.robot.locomotion.x + vx * math.cos(self.robot.locomotion.theta) * DT
        new_y = self.robot.locomotion.y + vx * math.sin(self.robot.locomotion.theta) * DT
        self.robot.locomotion.reposition_robot(new_x, new_y, new_theta)
        self.robot.locomotion.current_speed = Speed(vx, 0, vw)


if __name__ == '__main__':
    r = robot.Robot(behavior.Behaviors.Slave.value, ivy_address="127.0.0.255:2010", teensy_serial_path="/dev/ttyUSB0")
    r.locomotion.reposition_robot(500, 600, 0)
    r.locomotion.follow_trajectory(
        [(500, 680, 0), (600, 600, 0), (650, 500, 0), (550, 390, 0), (500, 100, 0)])
    app = QApplication(sys.argv)
    #start = timeit.default_timer()
    ex = App(r)
    #app.aboutToQuit.connect(ex.on_quit)
    #label = QLabel()
    #label.setPixmap(QPixmap.fromImage(ex))
    #label.show()
    #ex.dump_obstacle_grid_to_file("test.txt")
    #stop = timeit.default_timer()
    #print (stop - start)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
