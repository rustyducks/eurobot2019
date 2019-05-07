import enum
from locomotion.utils import *
from locomotion.params import ADMITTED_POSITION_ERROR, ROTATION_ACCELERATION_MAX, ADMITTED_ANGLE_ERROR, \
    ROTATION_SPEED_MAX, LINEAR_SPEED_MAX, ACCELERATION_MAX, LOOKAHEAD_DISTANCE


class PositionControl(LocomotionControlBase):
    class eState(enum.Enum):
        IDLE = 0
        FIRST_ROTATION = 1
        CRUISING = 2
        LAST_ROTATION = 3

    def __init__(self, robot):
        super().__init__(robot)
        self.state = self.eState.IDLE
        self.trajectory = []  # type: list[TrajPoint]
        self.goal_point = None
        self.pure_pursuit_traj_index = 1

    def reset_trajectory(self):
        self.trajectory.clear()
        self.state = self.eState.IDLE
        self.goal_point = None
        self.pure_pursuit_traj_index = 1

    def new_trajectory(self, points_list):
        self.reset_trajectory()
        self.state = self.eState.FIRST_ROTATION
        self.trajectory.append(TrajPoint(PointOrient(self.x, self.y, self.theta), 0))
        if len(points_list) > 0:
            for i, pt in enumerate(points_list):
                if i != len(points_list) - 1:
                    xe = points_list[i + 1][0]
                    ye = points_list[i + 1][1]

                    if i == 0:
                        xs = self.x
                        ys = self.y
                    else:
                        xs = points_list[i - 1][0]
                        ys = points_list[i - 1][1]
                    so = Vector2(pt[0] - xs, pt[1] - ys)
                    oe = Vector2(xe - pt[0], ye - pt[1])
                    if so.norm() == 0 or oe.norm() == 0:
                        continue
                    angle = math.acos(so.dot(oe) / (so.norm() * oe.norm()))
                    if abs(angle) >= math.pi / 2:
                        goal_speed = 0.
                    else:
                        goal_speed = LINEAR_SPEED_MAX * (1 - abs(angle) / (math.pi / 2))
                else:
                    goal_speed = 0.
                self.trajectory.append(TrajPoint(PointOrient(pt[0], pt[1], pt[2]), goal_speed))

    def compute_speed(self, delta_time):
        if self.state == self.eState.IDLE:
            return Speed(0, 0, 0)  # Maybe stay on the last point ?
        if self.state == self.eState.FIRST_ROTATION:
            aiming_angle = math.atan2(self.trajectory[1].point.y - self.y, self.trajectory[1].point.x - self.x)
            # print("Aiming angle: ",aiming_angle,"\tcurrent angle: ", self.current_pose.theta, "\tdiff : ", center_radians(
            #                   aiming_angle - self.current_pose.theta))
            if abs(self.current_speed.vtheta) <= 0.01 and abs(center_radians(
                    aiming_angle - self.current_pose.theta)) <= ADMITTED_ANGLE_ERROR:
                self.state = self.eState.CRUISING
            else:
                return self.rotation_only_loop(delta_time, aiming_angle)
        if self.state == self.eState.CRUISING:
            next_traj_point = self.trajectory[self.pure_pursuit_traj_index]
            position_error = self.current_pose.lin_distance_to_point(next_traj_point.point)
            speed_error = abs(self.current_speed.vx - next_traj_point.speed)
            if position_error <= ADMITTED_POSITION_ERROR and speed_error <= 0.1:
                # If we are not to far, at the right speed
                if self.pure_pursuit_traj_index == len(self.trajectory) - 1:
                    # If it is the last point in the trajectory: start final rotation
                    for i in range(self.pure_pursuit_traj_index):
                        self.trajectory.pop(0)
                    self.pure_pursuit_traj_index = 1
                    self.state = self.eState.LAST_ROTATION
                elif next_traj_point.speed <= 0.1:
                    # if it is not the last point, but the speed is 0: start a new initial rotation
                    for i in range(self.pure_pursuit_traj_index):
                        self.trajectory.pop(0)
                    self.pure_pursuit_traj_index = 1
                    self.state = self.eState.FIRST_ROTATION
                    aiming_angle = math.atan2(self.trajectory[1].point.y - self.y, self.trajectory[1].point.x - self.x)
                    return self.rotation_only_loop(delta_time, aiming_angle)
                else:  # just go to the next point in cruising
                    for i in range(self.pure_pursuit_traj_index):
                        self.trajectory.pop(0)
                    self.pure_pursuit_traj_index = 1
                    return self.pure_pursuit_loop(delta_time)
            else:
                speed = self.pure_pursuit_loop(delta_time)
                if abs(speed.vx) <= 0.1:
                    aiming_angle = math.atan2(self.goal_point.y - self.y, self.goal_point.x - self.x)
                    return self.rotation_only_loop(delta_time, aiming_angle)
                else:
                    return speed
        if self.state == self.eState.LAST_ROTATION:
            if self.current_speed.vtheta <= 0.01 and abs(center_radians(
                    self.trajectory[0].point.theta - self.current_pose.theta)) <= ADMITTED_ANGLE_ERROR:
                self.trajectory.pop(0)
                if len(self.trajectory) == 0:
                    self.state = self.eState.IDLE
                    print("[Position control] Trajectory ended.")
                    return Speed(0, 0, 0)
                else:
                    self.state = self.eState.FIRST_ROTATION
                    return self.rotation_only_loop(delta_time, self.trajectory[0].point.theta)
            else:
                return self.rotation_only_loop(delta_time, self.trajectory[0].point.theta)
        return Speed(0, 0, 0)

    def rotation_only_loop(self, delta_time, aiming_angle):
        diff = center_radians(aiming_angle - self.current_pose.theta)
        t_rotation_stop = abs(self.current_speed.vtheta) / ROTATION_ACCELERATION_MAX
        if diff < 0:
            omega = max((-ROTATION_SPEED_MAX, self.current_speed.vtheta - delta_time * ROTATION_ACCELERATION_MAX))
            planned_stop_angle = center_radians(self.current_pose.theta + 2 * (
                    self.current_speed.vtheta * t_rotation_stop + 1 / 2 * ROTATION_ACCELERATION_MAX * t_rotation_stop ** 2))
        else:
            omega = min((ROTATION_SPEED_MAX, self.current_speed.vtheta + delta_time * ROTATION_ACCELERATION_MAX))
            planned_stop_angle = center_radians(self.current_pose.theta + 2 * (
                    self.current_speed.vtheta * t_rotation_stop - 1 / 2 * ROTATION_ACCELERATION_MAX * t_rotation_stop ** 2))
        self.robot.ivy.highlight_robot_angle(0, aiming_angle)
        self.robot.ivy.highlight_robot_angle(1, planned_stop_angle)
        planned_angular_error = center_radians(aiming_angle - planned_stop_angle)
        if abs(planned_angular_error) <= ADMITTED_ANGLE_ERROR or diff * planned_angular_error < 0:
            if self.current_speed.vtheta > 0:
                omega = max((0, self.current_speed.vtheta - ROTATION_ACCELERATION_MAX * delta_time))
            else:
                omega = min((0, self.current_speed.vtheta + ROTATION_ACCELERATION_MAX * delta_time))
        elif abs(planned_angular_error) <= 3 * ADMITTED_ANGLE_ERROR:
            omega = max((ROTATION_ACCELERATION_MAX * delta_time, self.current_speed.vtheta))

        # print(omega)
        return Speed(0, 0, omega)

    def pure_pursuit_loop(self, delta_time):
        # Find the path point closest to the robot
        d = 99999999
        t_min = 1
        ith_traj_point = 0
        p_min = None
        for i in range(len(self.trajectory) - 1):
            ab = self.trajectory[i + 1].point - self.trajectory[i].point
            ar = self.robot.locomotion.current_pose - self.trajectory[i].point
            # print(ab)
            t = min(1, max(0, ar.dot(ab) / ab.squared_norm()))
            p = self.trajectory[i].point + ab * t
            pr = self.current_pose - p
            dist = pr.norm()

            # print(p, dist)
            if dist < d:
                d = dist
                t_min = t
                ith_traj_point = i
                p_min = p

            if i != 0 and self.trajectory[i].speed == 0:
                break

        # self.goal_point = p_min

        # Find the goal point
        lookhead_left = LOOKAHEAD_DISTANCE
        browse_traj_i = ith_traj_point
        # self.trajectory = self.trajectory[ith_traj_point:]
        t_robot = t_min
        # print(lookahead_t, t_min, goal_t)
        path_len = (self.trajectory[browse_traj_i + 1].point - p_min).norm()
        while lookhead_left > path_len:
            # Go to the next segment and recompute it while lookhead_left > 1. If last traj element is reach, extrapolate or clamp to 1 ?
            if self.trajectory[browse_traj_i + 1].speed == 0:
                t_robot = 0
                lookhead_left = (self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point).norm()
                break
            t_robot = 0
            browse_traj_i += 1
            lookhead_left -= path_len
            path_len = (self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point).norm()

        self.pure_pursuit_traj_index = browse_traj_i + 1  #  The next point of the trajectory we aim at.

        ab = self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point
        goal_t = t_robot + lookhead_left / ab.norm()
        goal_point = self.trajectory[browse_traj_i].point + ab * goal_t
        # print(lookhead_left, ab, goal_t)
        self.goal_point = goal_point

        # Goal point in vehicle coord
        goal_point_r = Point(
            (goal_point.x - self.x) * math.sin(-self.theta) + (goal_point.y - self.y) * math.cos(-self.theta),
            -(goal_point.x - self.x) * math.cos(-self.theta) + (goal_point.y - self.y) * math.sin(-self.theta))

        self.goal_point = goal_point
        # self.robot.ivy.highlight_point(2, goal_point.x, goal_point.y)

        # Compute the curvature gamma
        gamma = 2 * goal_point_r[0] / (LOOKAHEAD_DISTANCE ** 2)

        vx = self.compute_linear_speed(delta_time, self.trajectory[browse_traj_i + 1])
        vtheta = vx * gamma

        return Speed(vx, 0, vtheta)

    def compute_linear_speed(self, delta_time, next_point_traj):
        """

        :param delta_time:
        :type delta_time:
        :param next_point_traj:
        :type next_point_traj: TrajPoint
        :return:
        :rtype:
        """
        distance_to_objective = self.current_pose.lin_distance_to_point(next_point_traj.point)

        # Acceleration part
        rg = next_point_traj.point - self.current_pose
        unit_orient = Vector2(math.cos(self.theta), math.sin(self.theta))
        if (rg.norm() <= 0.001):
            alpha = math.pi / 2
        else:
            alpha = math.acos(rg.dot(unit_orient) / rg.norm())
        target_speed = LINEAR_SPEED_MAX * (1 - abs(alpha) / (math.pi / 3))
        current_linear_speed = self.current_speed.vx
        new_speed = min((max((target_speed, current_linear_speed - delta_time * ACCELERATION_MAX)),
                         current_linear_speed + delta_time * ACCELERATION_MAX))

        # Check if we need to decelerate
        if new_speed > next_point_traj.speed:
            t_reach_speed = (current_linear_speed - next_point_traj.speed) / ACCELERATION_MAX
            reach_speed_length = 2 * (
                    current_linear_speed * t_reach_speed - 1 / 2 * ACCELERATION_MAX * t_reach_speed ** 2)
            # Why 2 times ? Don't know, without the factor, it is largely underestimated...
            # Maybe because of the reaction time of the motors ?

            # current_speed_alpha = math.atan2(self.current_speed[1], self.current_speed[0])
            planned_stop_point = Point(self.x + reach_speed_length * math.cos(self.theta),
                                       self.y + reach_speed_length * math.sin(self.theta))
            # self.robot.ivy.highlight_point(1, planned_stop_point.x, planned_stop_point.y)
            planned_stop_error = planned_stop_point.lin_distance_to_point(next_point_traj.point)
            if planned_stop_error <= ADMITTED_POSITION_ERROR or planned_stop_point.lin_distance_to(self.x,
                                                                                                   self.y) > distance_to_objective:
                # Deceleration time
                new_speed = max((0, current_linear_speed - ACCELERATION_MAX * delta_time))
            elif planned_stop_error <= 3 * ADMITTED_POSITION_ERROR and abs(current_linear_speed) > 10:
                # Do not accelerate if we already plan to stop close to the point
                new_speed = current_linear_speed

        return new_speed
