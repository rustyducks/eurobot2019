import math
import time
from collections import namedtuple


ACCELERATION_MAX = 20   # mm/s²
LINEAR_SPEED_MAX = 150  # mm/s
ADMITTED_POSITION_ERROR = 50  # mm

def center_radians(value):
    while value <= - math.pi:
        value += 2 * math.pi
    while value > math.pi:
        value -= 2 * math.pi
    return value


class Locomotion:
    def __init__(self, robot):
        self.robot = robot
        self.x = 0
        self.y = 0
        self.theta = 0
        self.current_point_objective = None  # type: self.PointOrient
        self.current_speed = (0, 0, 0)
        self._last_position_control_time = None
        self._odometry_reports = {}  # type: dict[(int, int): (float, float, float)]
        self._latest_odometry_report = 0

    def handle_new_odometry_report(self, old_report_id, new_report_id, dx, dy, dtheta):
        if (new_report_id - self._latest_odometry_report + 256) % 256 < 128:
            if old_report_id - self._latest_odometry_report > 128:
                # Need to find previous report and add the new information
                for ids, deltas in list(self._odometry_reports.items()):
                    if ids[0] == old_report_id and ids[1] == self._latest_odometry_report:
                        self.x -= deltas[0]
                        self.y -= deltas[1]
                        self.theta = center_radians(self.theta - deltas[2])
                        break
                #  in fact we search for old_report_id -> _latest_odemtry_report, but what we need is
                #  a CHAIN going from old_report_id to _latest_odemetry_report and substract each node (and add them if
                # they overlap).

                self.x += dx
                self.y += dy
                self.theta = center_radians(self.theta + dtheta)

                self._odometry_reports[(old_report_id, new_report_id)] = (dx, dy, dtheta)
            elif old_report_id == self._latest_odometry_report:
                # Nominal case, the new report brings only new information
                self.x += dx
                self.y += dy
                self.theta = center_radians(self.theta + dtheta)

                self._odometry_reports[(old_report_id, new_report_id)] = (dx, dy, dtheta)
            else:
                # Should not happen, information has been missed
                pass
            self._latest_odometry_report = new_report_id
        elif new_report_id == self._latest_odometry_report:
            # We already have this information, but it may worth to update it
            pass
        else:
            # Only old information; it probably don't worth to update it...
            pass

        #  Delete all the old reports (those taken into account by the teensy)
        for ids, delta in list(self._odometry_reports.items()):
            if (ids[1] - old_report_id + 256) % 256 == 0 or (ids[1] - old_report_id + 256) % 256 > 128:
                del(self._odometry_reports[ids])

    def go_to_orient(self, x, y, theta):
        self.current_point_objective = self.PointOrient(x, y, theta)

    def go_to_orient_point(self, point):
        self.go_to_orient(point.x, point.y, point.theta)

    def position_control_loop(self):
        control_time = time.time()
        if self._last_position_control_time is None:
            delta_time = 0
        else:
            delta_time = control_time - self._last_position_control_time
        self._last_position_control_time = control_time

        if self.current_point_objective is not None:
            self.robot.ivy.highlight_point(0, self.current_point_objective.x, self.current_point_objective.y)
            distance_to_objective = self.current_point_objective.lin_distance_to(self.x, self.y)
            # if distance_to_objective <= ADMITTED_POSITION_ERROR:
            #     self.current_point_objective = None
            #     return

            # Acceleration part
            alpha = math.atan2(self.current_point_objective.y - self.y, self.current_point_objective.x - self.x)
            current_linear_speed = math.hypot(self.current_speed[0], self.current_speed[1])
            new_speed = min((LINEAR_SPEED_MAX, current_linear_speed + delta_time * ACCELERATION_MAX))

            # Check if we need to decelerate
            t_stop = current_linear_speed / ACCELERATION_MAX
            stop_length = 2 * (current_linear_speed * t_stop - 1/2 * ACCELERATION_MAX * t_stop ** 2)  # Why 2 times ? Don't know, without the factor, it is largely underestimated... Maybe because of the reaction time of the motors ?
            # current_speed_alpha = math.atan2(self.current_speed[1], self.current_speed[0])
            planned_stop_point = self.Point(self.x + stop_length * math.cos(alpha), self.y + stop_length * math.sin(alpha))
            self.robot.ivy.highlight_point(1, planned_stop_point.x, planned_stop_point.y)
            planned_stop_error = planned_stop_point.lin_distance_to_point(self.current_point_objective)
            if planned_stop_error <= ADMITTED_POSITION_ERROR or planned_stop_point.lin_distance_to(self.x, self.y) > distance_to_objective:
                # Deceleration time
                new_speed = max((0, current_linear_speed - ACCELERATION_MAX * delta_time))

            self.current_speed = (new_speed * math.cos(alpha), new_speed * math.sin(alpha), 0)

        else:
            self.current_speed = (0, 0, 0)
        print("Speed : " + str(self.current_speed))
        self.robot.communication.send_speed_command(*self.current_speed)


    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def lin_distance_to_point(self, pt):
            return self.lin_distance_to(pt.x, pt.y)

        def lin_distance_to(self, x, y):
            return math.hypot(x - self.x, y - self.y)

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta
