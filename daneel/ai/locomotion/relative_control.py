from enum import Enum

from locomotion.utils import LocomotionControlBase, Speed, PointOrient, center_radians, Point

from locomotion.params import ADMITTED_POSITION_ERROR, ADMITTED_ANGLE_ERROR, ACCELERATION_MAX, \
    ROTATION_ACCELERATION_MAX, ROTATION_SPEED_MAX


class RelativeControl(LocomotionControlBase):
    class eState(Enum):
        IDLE = 0
        STRAIGHT = 1
        ROTATE = 2

    def __init__(self, robot):
        super().__init__(robot)
        self.state = self.eState.IDLE
        self._start_position = PointOrient(0, 0, 0)
        self._relative_straight_goal = 0
        self._rotation_goal = 0

    def compute_speed(self, delta_time, speed_constraints):
        if self.state == self.eState.IDLE:
            return Speed(0, 0, 0)
        elif self.state == self.eState.STRAIGHT:
            if abs(self._relative_straight_goal) <= ADMITTED_POSITION_ERROR:
                self.state = self.eState.IDLE
                return Speed(0, 0, 0)
            else:
                dist_remaining = abs(self._relative_straight_goal) - self._start_position.lin_distance_to_point(self.current_pose)
                if self._relative_straight_goal <0:
                    dist_remaining = -dist_remaining
                if abs(dist_remaining) <= ADMITTED_POSITION_ERROR:
                    self.state = self.eState.IDLE
                    return Speed(0, 0, 0)
                else:
                    return self.linear_speed(speed_constraints, dist_remaining, delta_time)
        elif self.state == self.eState.ROTATE:
            if abs(center_radians(self.theta - self._rotation_goal)) <= ADMITTED_ANGLE_ERROR:

                self.state = self.eState.IDLE
                return Speed(0, 0, 0)
            else:
                return self.rotational_speed(speed_constraints, delta_time)

    def new_straight_goal(self, relative_goal):
        self._relative_straight_goal = relative_goal
        self._start_position = Point(self.current_pose.x, self.current_pose.y)
        self.state = self.eState.STRAIGHT

    def new_rotate_goal(self, relative_goal):
        if abs(relative_goal) <= ADMITTED_ANGLE_ERROR:
            return
        self._rotation_goal = center_radians(self.current_pose.theta + relative_goal)
        self.state = self.eState.ROTATE

    def linear_speed(self, speed_constraints, dist_remaining, dt):
        # Acceleration part
        if dist_remaining < 0:
            acceleration = -ACCELERATION_MAX
        else:
            acceleration = ACCELERATION_MAX

        mul = 1
        if abs(dist_remaining) <= 3 * ADMITTED_POSITION_ERROR:
            mul = 0.2
        new_speed = self.current_speed.vx + acceleration * dt * mul

        # Check if we need to decelerate
        t_stop = abs(self.current_speed.vx / ACCELERATION_MAX)
        reach_speed_length = 2 * (
                self.current_speed.vx * t_stop - 1 / 2 * acceleration * t_stop ** 2)
        # Why 2 times ? Don't know, without the factor, it is largely underestimated...
        # Maybe because of the reaction time of the motors ?
        planned_stop_error = abs(reach_speed_length - dist_remaining)
        if planned_stop_error <= ADMITTED_POSITION_ERROR:
            # Deceleration time
            speed_diff = acceleration * dt
            if abs(speed_diff) > abs(self.current_speed.vx):
                new_speed = 0
            else:
                new_speed = self.current_speed.vx - acceleration * dt
        elif planned_stop_error <= 3 * ADMITTED_POSITION_ERROR and abs(self.current_speed.vx) > 10:
            # Do not accelerate if we already plan to stop close to the point
            new_speed = self.current_speed.vx

        new_speed = min(speed_constraints.max_vx, max(speed_constraints.min_vx, new_speed))
        return Speed(new_speed, 0, 0)

    def rotational_speed(self, speed_constraints, dt):
        diff = center_radians(self._rotation_goal - self.current_pose.theta)
        if diff < 0:
            acceleration = -ROTATION_ACCELERATION_MAX
        else:
            acceleration = ROTATION_ACCELERATION_MAX
        t_rotation_stop = abs(self.current_speed.vtheta / acceleration)
        mul = 1
        if abs(diff) <= 3 * ADMITTED_ANGLE_ERROR:
            mul = 0.2
        omega = self.current_speed.vtheta + dt * acceleration * mul
        planned_stop_angle = center_radians(self.current_pose.theta + 2 * (
                self.current_speed.vtheta * t_rotation_stop - 1 / 2 * acceleration * t_rotation_stop ** 2))
        self.robot.ivy.highlight_robot_angle(0, self._rotation_goal)
        self.robot.ivy.highlight_robot_angle(1, planned_stop_angle)
        planned_angular_error = center_radians(self._rotation_goal - planned_stop_angle)
        if abs(planned_angular_error) <= ADMITTED_ANGLE_ERROR or diff * planned_angular_error < 0:
            speed_diff = acceleration * dt
            if abs(speed_diff) > abs(self.current_speed.vtheta):
                omega = 0
            else:
                omega = self.current_speed.vtheta - speed_diff
        elif abs(planned_angular_error) <= 3 * ADMITTED_ANGLE_ERROR and abs(self.current_speed.vtheta) > 0.3:
            omega = self.current_speed.vtheta

        omega = min(speed_constraints.max_vtheta, max(speed_constraints.min_vtheta, omega))
        # print(omega)
        return Speed(0, 0, omega)
