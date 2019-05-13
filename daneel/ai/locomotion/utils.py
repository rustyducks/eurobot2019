from typing import NamedTuple
import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def lin_distance_to_point(self, pt):
        return self.lin_distance_to(pt.x, pt.y)

    def lin_distance_to(self, x, y):
        return math.hypot(x - self.x, y - self.y)

    def __add__(self, other):
        if isinstance(other, Vector2):
            return Point(self.x + other.x, self.y + other.y)
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __getitem__(self, item):
        if item == 0 or item == 'x':
            return self.x
        elif item == 1 or item == 'y':
            return self.y

    def __setitem__(self, key, value):
        if key == 0 or key == 'x':
            self.x = value
        elif key == 1 or key == 'y':
            self.y = value

    def __repr__(self):
        return "Point({}, {})".format(self.x, self.y)


class PointOrient(Point):
    def __init__(self, x, y, theta=0):
        super().__init__(x, y)
        self.theta = theta

    def __getitem__(self, item):
        if item == 2 or item == "theta":
            return self.theta
        else:
            return super(PointOrient, self).__getitem__(item)
        
    def __setitem__(self, key, value):
        if key == 2 or key == "theta":
            self.theta = value
        else:
            return super(PointOrient, self).__setitem__(key, value)


class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self):
        return math.hypot(self.x, self.y)

    def squared_norm(self):
        return self.x ** 2 + self.y ** 2

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def __getitem__(self, item):
        if item == 0 or item == 'x':
            return self.x
        elif item == 1 or item == 'y':
            return self.y

    def __setitem__(self, key, value):
        if key == 0 or key == 'x':
            self.x = value
        elif key == 1 or key == 'y':
            self.y = value

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return "Vector2({}, {})".format(self.x, self.y)


Speed = NamedTuple("Speed", [('vx', float), ('vy', float), ('vtheta', float)])
SpeedConstraint = NamedTuple("SpeedConstraint", [('min_vx', float), ('max_vx', float), ('min_vy', float),
                                                 ('max_vy', float), ('min_vtheta', float), ('max_vtheta', float)])
TrajPoint = NamedTuple("GoalPoint", [('point', PointOrient), ('speed', float)])

def center_radians(value):
    while value < - math.pi:
        value += 2 * math.pi
    while value >= math.pi:
        value -= 2 * math.pi
    return value


class LocomotionControlBase:
    def __init__(self, robot):
        self.robot = robot

    def compute_speed(self, delta_time, speed_constraints):
        """

        :param delta_time:
        :type delta_time:
        :param speed_constraints:
        :type speed_constraints: SpeedConstraint
        :return:
        :rtype:
        """
        print("Warning: LocomotionControlBase compute_speed function must be overloaded. Setting speeds to 0")
        return Speed(0, 0, 0)

    @property
    def x(self):
        return self.robot.locomotion.current_pose.x

    @property
    def y(self):
        return self.robot.locomotion.current_pose.y

    @property
    def theta(self):
        return self.robot.locomotion.current_pose.theta

    @property
    def current_pose(self):
        """

        :return:
        :rtype: PointOrient
        """
        return self.robot.locomotion.current_pose

    @property
    def current_speed(self):
        return self.robot.locomotion.current_speed
