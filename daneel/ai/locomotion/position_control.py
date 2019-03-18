import enum
from locomotion.utils import *
from locomotion.params import ADMITTED_POSITION_ERROR, ROTATION_ACCELERATION_MAX, ADMITTED_ANGLE_ERROR, \
    ROTATION_SPEED_MAX, LINEAR_SPEED_MAX, ACCELERATION_MAX

LOOKAHEAD_DISTANCE = 100.

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

    def compute_speed(self, delta_time):
        if self.state == self.eState.IDLE:
            return Speed(0, 0, 0)  # Maybe stay on the last point ?
        if self.state == self.eState.FIRST_ROTATION:
            aiming_angle = math.atan2(self.trajectory[1].point.y - self.y, self.trajectory[1].point.x - self.x)
            print("Aiming angle: ",aiming_angle,"\tcurrent angle: ", self.current_pose.theta, "\tdiff : ", center_radians(
                            aiming_angle - self.current_pose.theta))
            if abs(self.current_speed.vtheta) <= 0.01 and abs(center_radians(
                            aiming_angle - self.current_pose.theta)) <= ADMITTED_ANGLE_ERROR:
                self.state = self.eState.CRUISING
            else:
                return self.rotation_only_loop(delta_time, aiming_angle)
        if self.state == self.eState.CRUISING:
            position_error = self.current_pose.lin_distance_to_point(self.trajectory[1].point)
            speed_error = abs(self.current_speed.vx - self.trajectory[1].speed)
            if position_error <= ADMITTED_POSITION_ERROR and speed_error <= 0.1:
                print("Plop")
                # If we are not to far, at the right speed
                if len(self.trajectory) == 1:  # If it is the last point in the trajectory: start final rotation
                    self.state = self.eState.LAST_ROTATION
                elif self.trajectory[1].speed <= 0.1:
                    # if it is not the last point, but the speed is 0: start a new initial rotation
                    print("################POP")
                    self.trajectory.pop(0)
                    self.state = self.eState.FIRST_ROTATION
                    print("Rotation only \n")
                    aiming_angle = math.atan2(self.trajectory[1].point.y - self.y, self.trajectory[1].point.x - self.x)
                    return self.rotation_only_loop(delta_time, aiming_angle)
                else:  # just go to the next point in cruising
                    print("################POP")
                    self.trajectory.pop(0)
                    return self.pure_pursuit_loop(delta_time)
            else:
                speed = self.pure_pursuit_loop(delta_time)
                if abs(speed.vx) <= 0.1:
                    aiming_angle = math.atan2(self.goal_point.y - self.y, self.goal_point.x - self.x)
                    return self.rotation_only_loop(delta_time, aiming_angle)
                else:
                    return speed
        if self.state == self.eState.LAST_ROTATION:
            if self.current_speed.vtheta <= 0.01 and center_radians(
                            self.trajectory[1].point.theta - self.current_pose.theta) <= ADMITTED_ANGLE_ERROR:
                print("################POP")
                self.trajectory.pop(0)
                if len(self.trajectory) == 1:
                    self.state = self.eState.IDLE
                    return Speed(0, 0, 0)
                else:
                    self.state = self.eState.FIRST_ROTATION
                    return self.rotation_only_loop(delta_time, self.trajectory[1].point.theta)
            else:
                return self.rotation_only_loop(delta_time, self.trajectory[1].point.theta)
        return Speed(0, 0, 0)

    def rotation_only_loop(self, delta_time, aiming_angle):
        omega = min((ROTATION_SPEED_MAX, abs(self.current_speed.vtheta) + delta_time * ROTATION_ACCELERATION_MAX))
        rotation_error_sign = math.copysign(1, center_radians(aiming_angle - self.current_pose.theta))
        t_rotation_stop = abs(self.current_speed.vtheta) / ROTATION_ACCELERATION_MAX
        planned_stop_angle = center_radians(self.current_pose.theta + rotation_error_sign * 2.5 * (
            abs(
                self.current_speed.vtheta) * t_rotation_stop - 1 / 2 * ROTATION_ACCELERATION_MAX * t_rotation_stop**2))
        self.robot.ivy.highlight_robot_angle(0, aiming_angle)
        self.robot.ivy.highlight_robot_angle(1, planned_stop_angle)
        planned_angular_error = abs(center_radians(planned_stop_angle - aiming_angle))
        if planned_angular_error <= ADMITTED_ANGLE_ERROR or abs(center_radians(planned_stop_angle - self.theta)) > abs(
                center_radians(aiming_angle - self.theta)):
            omega = max((0, abs(self.current_speed.vtheta) - ROTATION_ACCELERATION_MAX * delta_time))
        return Speed(0, 0, math.copysign(omega, rotation_error_sign))

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
            self.ab = ab
            self.ar = ar
            self.pro = p

            # print(p, dist)
            if dist < d:
                d = dist
                t_min = t
                ith_traj_point = i
                p_min = p

        print("min dist: ", d, "i:", ith_traj_point)


        # self.goal_point = p_min

        # Find the goal point
        lookhead_left = LOOKAHEAD_DISTANCE
        browse_traj_i = ith_traj_point
        #self.trajectory = self.trajectory[ith_traj_point:]
        t_robot = t_min
        # print(lookahead_t, t_min, goal_t)
        path_len = (self.trajectory[browse_traj_i + 1].point - p_min).norm()
        while lookhead_left > path_len:
            # Go to the next segment and recompute it while lookhead_left > 1. If last traj element is reach, extrapolate or clamp to 1 ?
            t_robot = 0
            browse_traj_i += 1
            lookhead_left -= path_len
            path_len = (self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point).norm()

        ab = self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point
        goal_t = t_robot + lookhead_left / ab.norm()
        goal_point = self.trajectory[browse_traj_i].point + ab * goal_t
        # print(lookhead_left, ab, goal_t)
        self.goal_point = goal_point

        # Goal point in vehicle coord
        goal_point_r = Point((goal_point.x - self.x) * math.sin(-self.theta) + (goal_point.y - self.y) * math.cos(-self.theta),
                             -(goal_point.x - self.x) * math.cos(-self.theta) + (goal_point.y - self.y) * math.sin(-self.theta))

        self.goal_point = goal_point
        self.robot.ivy.highlight_point(2, goal_point.x, goal_point.y)

        # Compute the curvature gamma
        gamma = 2 * goal_point_r[0] / (LOOKAHEAD_DISTANCE ** 2)
        print("gamma:", gamma, "radius:", 1/gamma)

        vx = 100
        vtheta = vx * gamma

        return Speed(vx, 0, vtheta)



        # #self.robot.ivy.highlight_point(0, self.trajectory[0].point.x, self.trajectory[0].point.y)
        #
        # # Pure pursuit
        # # Find the path point closest to the robot
        # distances = [tp.point.lin_distance_to_point(self.current_pose) for tp in self.trajectory]
        # #min_index, min_dist = min(enumerate(distances), key=lambda d: d[1])
        #
        #
        # # self.goal_point = p_min
        #
        # # Find the goal point
        # lookhead_left = LOOKAHEAD_DISTANCE
        # browse_traj_i = 0
        # # print(lookahead_t, t_min, goal_t)
        # path_len = self.trajectory[self.trajectory_index].point.lin_distance_to_point(self.current_pose)
        # while lookhead_left > path_len:
        #     # Go to the next segment and recompute it while lookhead_left > 1. If last traj element is reach, extrapolate or clamp to 1 ?
        #     browse_traj_i += 1
        #     lookhead_left -= path_len
        #     if browse_traj_i == len(self.trajectory)-1:
        #         break
        #     path_len = self.trajectory[browse_traj_i + 1].point.lin_distance_to_point(self.trajectory[browse_traj_i].point)
        #
        # if browse_traj_i == len(self.trajectory) - 1:
        #     goal_point = self.trajectory[0].point
        # else:
        #     ab = self.trajectory[browse_traj_i + 1].point - self.trajectory[browse_traj_i].point
        #     goal_t = lookhead_left / ab.norm()
        #     goal_point = self.trajectory[browse_traj_i].point + Vector2(goal_t * ab.x, goal_t * ab.y)
        #
        # self.goal_point = goal_point  # For debugging...
        # #self.robot.ivy.highlight_point(2, goal_point.x, goal_point.y)
        # # print(lookhead_left, ab, goal_t)
        # # self.goal_point = goal_point
        #
        # # Goal point in vehicle coord
        # goal_point_r = -(goal_point.x - self.x) * math.sin(self.theta) + (goal_point.y - self.y) * math.cos(self.theta)
        #
        # distance_to_objective = self.current_pose.lin_distance_to(goal_point.x, goal_point.y)
        #
        # # Acceleration part
        # rg = goal_point - self.current_pose
        # unit_orient = Vector2(math.cos(self.theta), math.sin(self.theta))
        # alpha = math.acos((rg.x * unit_orient.x + rg.y * unit_orient.y) / rg.norm())
        # target_speed = LINEAR_SPEED_MAX * (1 - abs(alpha) / (math.pi / 2))
        # current_linear_speed = math.hypot(self.current_speed.vx, self.current_speed.vy)
        # new_speed = min((target_speed, current_linear_speed + delta_time * ACCELERATION_MAX))
        # print(new_speed)
        # print(self.trajectory)
        #
        # # Check if we need to decelerate
        # if new_speed > self.trajectory[0].speed:
        #     t_reach_speed = (current_linear_speed - self.trajectory[0].speed) / ACCELERATION_MAX
        #     reach_speed_length = 2 * (
        #         current_linear_speed * t_reach_speed - 1 / 2 * ACCELERATION_MAX * t_reach_speed ** 2)
        #     # Why 2 times ? Don't know, without the factor, it is largely underestimated...
        #     # Maybe because of the reaction time of the motors ?
        #
        #     # current_speed_alpha = math.atan2(self.current_speed[1], self.current_speed[0])
        #     planned_stop_point = Point(self.x + reach_speed_length * math.cos(self.theta),
        #                                self.y + reach_speed_length * math.sin(self.theta))
        #     self.robot.ivy.highlight_point(1, planned_stop_point.x, planned_stop_point.y)
        #     planned_stop_error = planned_stop_point.lin_distance_to_point(self.trajectory[0].point)
        #     if planned_stop_error <= ADMITTED_POSITION_ERROR or planned_stop_point.lin_distance_to(self.x,
        #                                                                                            self.y) > distance_to_objective:
        #         # Deceleration time
        #         new_speed = max((0, current_linear_speed - ACCELERATION_MAX * delta_time))
        #
        #
        # # Compute the curvature gamma
        # gamma = 2 * goal_point_r / (LOOKAHEAD_DISTANCE ** 2)
        # vtheta = new_speed * gamma
        # speed_command = Speed(new_speed, 0, vtheta)
        # print(speed_command)
        #
        # return speed_command

