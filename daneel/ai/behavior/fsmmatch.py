from enum import Enum
import time
import math
from math import pi
import os
import armothy
from table.table import SlotName, Atom, ScoreInScale, ChaosZone
from robot_parts import AtomStorage

from behavior import Behavior

from locomotion.utils import center_radians, Point

END_MATCH_TIME = 100  # in seconds
START_EXPERIMENT_TIME = 10  # in seconds

WARNING_VOLTAGE_THRESHOLD = 12.8  # if signal or power voltage goes under this value, led will be flashing red.




class Color(Enum):
    YELLOW = "yellow"
    PURPLE = "purple"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self._score = 0
        self.state = StatePreStartChecks(self)
        self.shutdown_button_press_time = 0
        self.last_start_experiment = 0

    @property
    def score(self):
        return self._score

    @score.setter
    def score(self, v):
        self._score = v
        self.robot.io.score_display_number(self._score)

    def loop(self):
        time_now = time.time()
        if self.shutdown_button_press_time == 0 and self.robot.io.button2_state == self.robot.io.ButtonState.PRESSED:
            self.shutdown_button_press_time = time.time()
        elif self.shutdown_button_press_time != 0 and self.robot.io.button2_state == self.robot.io.ButtonState.RELEASED:
            self.shutdown_button_press_time = 0
        elif self.shutdown_button_press_time != 0 and time.time() - self.shutdown_button_press_time >= 5:
            self.robot.locomotion.stop()
            for i in range(3):
                self.robot.io.set_led_color(self.robot.io.LedColor.RED)
                time.sleep(0.5)  # Oh my god ! A "sleep" ! Don't worry baby, we are going to shut down any way.
                self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
                time.sleep(0.5)
            os.system("sudo shutdown -h now")
            exit(0)
        if self.start_time is not None and time_now - self.start_time >= END_MATCH_TIME and self.state.__class__ != StateEnd:
            print("[FSMMatch] End match")
            next_state = StateEnd
        else:
            next_state = self.state.test()
        if self.start_time is not None and time_now - self.start_time >= START_EXPERIMENT_TIME and self.state.__class__ != StateEnd:
            if self.last_start_experiment == 0:
                self.score += 30
                self.robot.io.score_display_number(self.score)
            if time_now - self.last_start_experiment >= 5:
                self.robot.io.launch_experiment()
                self.last_start_experiment = time_now
        if next_state is not None:
            print("[FSMMatch] Leaving {}, entering {}".format(self.state.__class__.__name__, next_state.__name__))
            self.state.deinit()
            self.state = next_state(self)

    def start_match(self):
        print("[FSMMatch] Match Started")
        self.start_time = time.time()


class FSMState:
    def __init__(self, behavior):
        self.behavior = behavior
        self.robot = self.behavior.robot

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


class StatePreStartChecks(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_POWER, self.robot.io.SensorState.PERIODIC)
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_SIGNAL,
                                               self.robot.io.SensorState.PERIODIC)
        self.robot.io.set_lidar_pwm(244)
        self.enter_time = time.time()
        self.robot.locomotion.set_direct_speed(0, 0, 0)

    def test(self):
        if self.robot.io.battery_power_voltage is not None and self.robot.io.battery_signal_voltage is not None:
            if (time.time() - self.enter_time) % 13 < 5:
                if self.robot.io.battery_signal_voltage <= WARNING_VOLTAGE_THRESHOLD and (
                        time.time() - self.enter_time) % 2 < 1:
                    self.robot.io.set_led_color(self.robot.io.LedColor.RED)
                else:
                    self.robot.io.set_led_color(self.robot.io.LedColor.CYAN)
                self.robot.io.score_display_number(round(self.robot.io.battery_signal_voltage * 100),
                                                   with_two_points=True)
            elif (time.time() - self.enter_time) % 13 < 10:
                if self.robot.io.battery_power_voltage <= WARNING_VOLTAGE_THRESHOLD and (
                        time.time() - self.enter_time) % 2 < 1:
                    self.robot.io.set_led_color(self.robot.io.LedColor.RED)
                else:
                    self.robot.io.set_led_color(self.robot.io.LedColor.ORANGE)
                self.robot.io.score_display_number(round(self.robot.io.battery_power_voltage * 100),
                                                   with_two_points=True)
            else:
                self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
                self.robot.io.score_display_rusty_ducks()

        if self.robot.io.button2_state == self.robot.io.ButtonState.PRESSED:
            return StateColorSelection

    def deinit(self):
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_POWER, self.robot.io.SensorState.STOPPED)
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_SIGNAL, self.robot.io.SensorState.STOPPED)
        self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
        self.robot.io.score_display_rusty_ducks()


class StateColorSelection(FSMState):
    class ColorState(Enum):
        IDLE = "idle"
        PRESSED = "pressed"

    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.color = Color.YELLOW
        self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.YELLOW)
        if self.robot.io.button1_state == self.robot.io.ButtonState.RELEASED and not self.behavior.color == Color.YELLOW:
            self.behavior.color = Color.YELLOW
            self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.YELLOW)
        elif self.robot.io.button1_state == self.robot.io.ButtonState.PRESSED and not self.behavior.color == Color.PURPLE:
            self.behavior.color = Color.PURPLE
            self.robot.io.set_led_color(self.robot.io.LedColor.PURPLE)
        self.robot.io.armothy.rotate_z_axis(300)
        self.robot.io.armothy.rotate_y_axis(0)

    def test(self):
        if self.robot.io.button1_state == self.robot.io.ButtonState.RELEASED and not self.behavior.color == Color.YELLOW:
            self.behavior.color = Color.YELLOW
            self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.YELLOW)
        elif self.robot.io.button1_state == self.robot.io.ButtonState.PRESSED and not self.behavior.color == Color.PURPLE:
            self.behavior.color = Color.PURPLE
            self.robot.io.set_led_color(self.robot.io.LedColor.PURPLE)

        if self.robot.io.cord_state == self.robot.io.CordState.IN:
            return StatePreMatch

    def deinit(self):
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.reposition_robot(145, 1255, 0.016305118295449166)
        else:
            self.robot.locomotion.reposition_robot(2855, 1255, -math.pi + 0.016305118295449166)
        self.robot.io.set_led_color(self.robot.io.LedColor.WHITE)


class StatePreMatch(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)

    def test(self):
        if self.robot.io.cord_state == self.behavior.robot.io.CordState.OUT:
            return StateFrontGreenPeriodic

    def deinit(self):
        self.behavior.start_match()
        self.behavior.score = 5  # Experiment is present
        self.robot.io.score_display_number(self.behavior.score)


class StateFrontGreenPeriodic(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_z_axis(0)
        self.robot.io.armothy.rotate_y_axis(0)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(305, 1250, 0)])
        else:
            self.robot.locomotion.follow_trajectory([(2695, 1250, -math.pi)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateGetColorFrontGreenPeriodicAtom

    def deinit(self):
        pass


class StateGetColorFrontGreenPeriodicAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_z_axis(0)
        self.robot.io.armothy.rotate_y_axis(0)
        self.start_time = time.time()
        self.time_out = 2  # sec

    def test(self):
        seen_color = self.robot.io.jevois.uniq_last_puck_color
        if seen_color is not None:
            print("[FSMMatch] Seen green atom is : ", seen_color)
            if self.behavior.color == Color.YELLOW:
                self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color = Atom.Color.RED
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color = Atom.Color.RED
                return StateTakeFirstAtom
            else:
                self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color = Atom.Color.RED
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color = Atom.Color.RED
                return StateTakeFirstAtom
        if time.time() - self.start_time >= self.time_out:
            return StateTakeFirstAtom

    def deinit(self):
        pass


class StateTakeFirstAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.store_side = None
        if self.behavior.color == Color.YELLOW:
            if self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color == Atom.Color.RED:
                #  We put the reds in the right stack, we probably do not want to put them in the scale
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
                self.store_side = AtomStorage.Side.RIGHT
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
                self.store_side = AtomStorage.Side.LEFT
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color == Atom.Color.RED:
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
                self.store_side = AtomStorage.Side.LEFT
            else:
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
                self.store_side = AtomStorage.Side.RIGHT
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.RUNNING_SAFE:
            if self.behavior.color == Color.YELLOW:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom)
            else:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom)
            return StateGoToFrontBluePeriodic

        if time.time() - self.start_time >= self.timeout:
            self.robot.io.armothy.home()
            # Do not add the atom to the atom tank
            # return StateGoToFrontBluePeriodic
            return StateGoToFrontBluePeriodic

    def deinit(self):
        pass


class StateGoToFrontBluePeriodic(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(500, 1250, -pi / 2), (500, 1145, -pi / 2)])
        else:
            self.robot.locomotion.follow_trajectory([(2500, 1250, -pi / 2), (2500, 1145, -pi / 2)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            if self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color is not None:
                # If we already know the atom color
                return StateTakeSecondAtom
            else:
                return StateGetColorFrontBluePeriodicAtom

    def deinit(self):
        pass


class StateGetColorFrontBluePeriodicAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.translate_z_axis(5)
        self.robot.io.armothy.rotate_z_axis(0)
        self.start_time = time.time()
        self.time_out = 2  # sec

    def test(self):
        seen_color = self.robot.io.jevois.uniq_last_puck_color
        if seen_color is not None:
            print("[FSMMatch] Seen blue atom is : ", seen_color)
            if self.behavior.color == Color.YELLOW:
                self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green is still there...
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color = Atom.Color.RED
                elif seen_color == "red" and self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color == Atom.Color.RED:
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color = Atom.Color.GREEN
                return StateTakeSecondAtom
            else:
                self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green is still there...
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color = Atom.Color.RED
                elif seen_color == "red" and self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color == Atom.Color.RED:
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color = Atom.Color.GREEN
                return StateTakeSecondAtom
        if time.time() - self.start_time >= self.time_out:
            return StateTakeSecondAtom

    def deinit(self):
        pass


class StateTakeSecondAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.store_side = None
        if self.behavior.color == Color.YELLOW:
            if self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color == Atom.Color.RED:
                #  We put the reds in the right stack, we probably do not want to put them in the scale
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color == Atom.Color.RED:
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
            else:
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.RUNNING_SAFE:
            if self.behavior.color == Color.YELLOW:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom)
            else:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom)
            return StateGoToFrontRedPeriodic

        if time.time() - self.start_time >= self.timeout:
            self.robot.io.armothy.home()
            # Do not add the atom to the atom tank
            return StateGoToFrontRedPeriodic

    def deinit(self):
        pass


class StateGoToFrontRedPeriodic(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(500, 1345, pi / 2)])
        else:
            self.robot.locomotion.follow_trajectory([(2500, 1345, pi / 2)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            if self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color is not None:
                # If we already know the atom color
                return StateTakeThirdAtom
            else:
                return StateGetColorFrontRedPeriodicAtom

    def deinit(self):
        pass


class StateGetColorFrontRedPeriodicAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.translate_z_axis(5)
        self.robot.io.armothy.rotate_z_axis(0)
        self.start_time = time.time()
        self.time_out = 2  # sec

    def test(self):
        seen_color = self.robot.io.jevois.uniq_last_puck_color
        if seen_color is not None:
            print("[FSMMatch] Seen red atom is : ", seen_color)
            if self.behavior.color == Color.YELLOW:
                self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green and blue are still there...
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color = Atom.Color.RED
                return StateTakeThirdAtom
            else:
                self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green and blue are still there...
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color = Atom.Color.RED
                return StateTakeThirdAtom
        if time.time() - self.start_time >= self.time_out:
            return StateTakeThirdAtom

    def deinit(self):
        pass


class StateTakeThirdAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.store_side = None
        if self.behavior.color == Color.YELLOW:
            if self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color == Atom.Color.RED:
                #  We put the reds in the right stack, we probably do not want to put them in the scale
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color == Atom.Color.RED:
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
            else:
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.RUNNING_SAFE:
            if self.behavior.color == Color.YELLOW:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom)
                if len(self.robot.left_storage.atoms) > 1:
                    return StateDropAllInRed
                else:
                    return StateDropRediums
            else:
                self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom)
                if len(self.robot.right_storage.atoms) > 1:
                    return StateDropAllInRed
                else:
                    return StateDropRediums

        if time.time() - self.start_time >= self.timeout:
            self.robot.io.armothy.home()
            # Do not add the atom to the atom tank
            if self.behavior.color == Color.YELLOW:
                if len(self.robot.left_storage.atoms) > 1:
                    return StateDropAllInRed
                else:
                    return StateDropRediums
            else:
                if len(self.robot.right_storage.atoms) > 1:
                    return StateDropAllInRed
                else:
                    return StateDropRediums

    def deinit(self):
        pass


class StateDropRediums(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.traj_finished = False
        self.atom1_dropped = False
        self.atom2_droping = False
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.go_to_orient(500, 1550, -pi)
        else:
            self.robot.locomotion.go_to_orient(2500, 1550, 0)
        self.start_time = time.time()
        self.drop_timeout = 5

    def test(self):
        if not self.traj_finished:
            if self.robot.locomotion.trajectory_finished:
                self.traj_finished = True
                if self.behavior.color == Color.YELLOW:
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(),
                                                   armothy.eStack.RIGHT_STACK, 100)
                else:
                    self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(),
                                                   armothy.eStack.LEFT_STACK, 100)
                self.start_time = time.time()
        else:
            status = self.robot.io.armothy.get_macro_status()
            if status == armothy.eMacroStatus.FINISHED:
                self.behavior.score += 6  # 6 pts per Red in Red periodic zone
                if self.behavior.color == Color.YELLOW:
                    self.robot.right_storage.pop()
                    if not self.robot.right_storage.is_empty:
                        self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(),
                                                       armothy.eStack.RIGHT_STACK,
                                                       (-1)**len(self.robot.right_storage.atoms) * 100)
                        self.start_time = time.time()
                    else:
                        return StateDisengageRedPeriodic
                else:
                    self.robot.left_storage.pop()
                    if not self.robot.left_storage.is_empty:
                        self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(),
                                                       armothy.eStack.LEFT_STACK,
                                                       (-1)**len(self.robot.left_storage.atoms) * 100)
                        self.start_time = time.time()
                    else:
                        return StateDisengageRedPeriodic
            elif status == armothy.eMacroStatus.ERROR:
                if self.behavior.color == Color.YELLOW:
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(),
                                                   armothy.eStack.RIGHT_STACK, 100)
                else:
                    self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(),
                                                   armothy.eStack.LEFT_STACK, 100)
            elif time.time() - self.start_time >= self.drop_timeout:
                return StateDisengageRedPeriodic

    def deinit(self):
        pass


class StateDropAllInRed(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.go_to_orient(500, 1550, -pi)
        else:
            self.robot.locomotion.go_to_orient(2500, 1550, 0)
        self.traj_finished = False
        self.emptying_side = None
        self.unknown_color = 0
        self.dropped = 0

    def test(self):
        if not self.traj_finished:
            if self.robot.locomotion.trajectory_finished:
                self.traj_finished = True
                if not self.robot.right_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.RIGHT
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(), armothy.eStack.RIGHT_STACK,
                                                   (-1)**len(self.robot.right_storage.atoms) * 100)
                elif not self.robot.left_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.LEFT
                    self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(), armothy.eStack.LEFT_STACK,
                                                   (-1)**len(self.robot.left_storage.atoms) * 100)
                else:
                    return StateDisengageRedPeriodic

        else:
            status = self.robot.io.armothy.get_macro_status()
            print(status)
            if status == armothy.eMacroStatus.FINISHED:
                color = self.robot.storages[self.emptying_side].top().color
                if color == Atom.Color.RED:
                    self.behavior.score += 6
                elif color == Atom.Color.GREEN:
                    self.behavior.score += 1
                else:
                    self.unknown_color += 1
                self.dropped += 1
                self.robot.storages[self.emptying_side].pop()
                if not self.robot.right_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.RIGHT
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(),
                                                   armothy.eStack.RIGHT_STACK,
                                                   (-1) ** len(self.robot.right_storage.atoms) * 100)
                elif not self.robot.left_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.LEFT
                    self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(), armothy.eStack.LEFT_STACK,
                                                   (-1) ** len(self.robot.left_storage.atoms) * 100)
                else:
                    return StateDisengageRedPeriodic
            elif status == armothy.eMacroStatus.ERROR:
                if not self.robot.right_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.RIGHT
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_drop_height(), armothy.eStack.RIGHT_STACK,
                                                   (-1)**len(self.robot.right_storage.atoms) * 100)
                elif not self.robot.left_storage.is_empty:
                    self.emptying_side = AtomStorage.Side.LEFT
                    self.robot.io.armothy.put_down(self.robot.left_storage.armothy_drop_height(), armothy.eStack.LEFT_STACK,
                                                   (-1)**len(self.robot.left_storage.atoms) * 100)
                else:
                    return StateDisengageRedPeriodic


    def deinit(self):
        if self.dropped == 3:
            if self.unknown_color == 3:
                self.behavior.score += 13  # Two reds and one green
            elif self.unknown_color == 2:
                self.behavior.score += 7   # One red and one green
            elif self.unknown_color == 1:
                self.behavior.score += 1  # This should not happen. It is probably a green
        elif self.dropped == 2:
            if self.unknown_color == 2:
                self.behavior.score += 9
            elif self.unknown_color == 1:
                self.behavior.score += 4
        elif self.dropped == 1:
            if self.unknown_color == 1:
                self.behavior.score += 5


class StateDisengageRedPeriodic(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(-100)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateGoFrontParticleAccelerator

    def deinit(self):
        pass


class StateGoFrontParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(1710, 1700, math.pi/2)])
            self.robot.io.armothy.rotate_z_axis(200)
        else:
            self.robot.locomotion.follow_trajectory([(1290, 1700, math.pi/2)])
            self.robot.io.armothy.rotate_z_axis(-200)
        self.robot.io.armothy.translate_z_axis(45)
        self.robot.io.armothy.rotate_y_axis(315)

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            # return StateEngageParticleAccelerator
            return StateRepositionningParticleAccelerator

    def deinit(self):
        pass


class StateRepositionningParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.translate_z_axis(0)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.rotate_z_axis(200)
        else:
            self.robot.io.armothy.rotate_z_axis(-200)
        self.robot.io.armothy.rotate_y_axis(315)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.start_repositioning(y_end=1835, theta_end = math.pi/2 - 0.022417768511335135)
        else:
            self.robot.locomotion.start_repositioning(y_end=1835, theta_end=math.pi/2 + 0.022417768511335135)

    def test(self):
        if self.robot.locomotion.repositionning_finished:
            return StateEngageParticleAccelerator2

    def deinit(self):
        pass


class StateEngageParticleAccelerator2(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(-60)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.rotate_z_axis(200)
        else:
            self.robot.io.armothy.rotate_z_axis(-200)
        self.robot.io.armothy.translate_z_axis(45)
        self.robot.io.armothy.rotate_y_axis(315)

    def test(self):
        if self.robot.locomotion.relative_command_finished and abs(45 - self.robot.io.armothy.prismatic_z_axis) < 3:
            return StateRollParticleAccelerator

    def deinit(self):
        pass


class StateEngageParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(45)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateRollParticleAccelerator


    def deinit(self):
        pass


class StateRollParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            #self.robot.io.armothy.rotate_z_axis(-60)
            self.robot.io.armothy.rotate_z_axis(0)
            self.robot.locomotion.turn(math.radians(-15))
        else:
            self.robot.io.armothy.rotate_z_axis(0)
            self.robot.locomotion.turn(math.radians(15))
        self.start_time = time.time()

    def test(self):
        if time.time() - self.start_time >= 1:
            return StateDisengageParticleAccelerator

    def deinit(self):
        self.behavior.score += 10  # One atom in the accelerator
        self.behavior.score += 10  # Goldenium revealed !
        self.robot.io.score_display_number(self.behavior.score)


class StateDisengageParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.has_turned = False
        self.robot.locomotion.turn(center_radians(math.pi / 2 - self.robot.locomotion.theta))

    def test(self):
        if not self.has_turned and self.robot.locomotion.relative_command_finished:
            self.robot.locomotion.go_straight(-50)
            self.has_turned = True

        elif self.has_turned and self.robot.locomotion.relative_command_finished:
            # return GoToChaosZone
            return StateGoToGoldenium

    def deinit(self):
        pass


class StateGoToGoldenium(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(2250, 1600, math.pi/2)])
        else:
            self.robot.locomotion.follow_trajectory([(750, 1600, math.pi/2)])
        self.robot.io.armothy.translate_z_axis(25)

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateEngageGoldenium

    def deinit(self):
        pass


class StateEngageGoldenium(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.has_turned = False
        self.robot.locomotion.turn(center_radians(math.pi / 2 - self.robot.locomotion.theta))

    def test(self):
        if self.robot.io.armothy.pressure <= 30:
            self.behavior.score += 20
            self.robot.locomotion.go_straight(-5)
            return StateDisengageGoldenium
        if self.robot.locomotion.relative_command_finished:
            if not self.has_turned:
                self.has_turned = True
                self.robot.io.armothy.rotate_y_axis(315)
                self.robot.io.armothy.rotate_z_axis(0)
                self.robot.io.armothy.close_valve()
                self.robot.io.armothy.start_pump()
                self.robot.locomotion.go_straight(150)
            else:
                return StateDisengageWithoutGoldenium

    def deinit(self):
        self.robot.io.score_display_number(self.behavior.score)


class StateDisengageGoldenium(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(1600 - self.robot.locomotion.current_pose.y)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateGoToScaleGoldenium

    def deinit(self):
        pass


class StateDisengageWithoutGoldenium(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_y_axis(0)
        self.robot.io.armothy.stop_pump()
        self.robot.locomotion.go_straight(1600 - self.robot.locomotion.current_pose.y)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateGoToChaosZone

    def deinit(self):
        pass


class StateGoToScaleGoldenium(FSMState):
    # TODO Purple
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_y_axis(0)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.rotate_z_axis(315)
            self.robot.locomotion.follow_trajectory([(1500, 1550, 0), (1400, 1400, 0), (1300, 1000, 0), (1272, 700, -math.pi/2)])
        else:
            self.robot.io.armothy.rotate_z_axis(-315)
            self.robot.locomotion.follow_trajectory([(1500, 1550, 0), (1600, 1400, 0), (1700, 1000, 0), (1692, 700, -math.pi/2)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateEngageScaleGoldenium

    def deinit(self):
        pass


class StateEngageScaleGoldenium(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.translate_z_axis(5)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.rotate_z_axis(100)
        else:
            self.robot.io.armothy.rotate_z_axis(-100)
        self.robot.io.armothy.rotate_y_axis(315)
        self.robot.locomotion.go_straight(130)
        self.release_time = None

    def test(self):
        if (self.robot.locomotion.relative_command_finished or self.robot.locomotion.is_one_drifting) and self.release_time is None:
            self.robot.io.armothy.open_valve()
            self.robot.io.armothy.stop_pump()
            self.behavior.score += 24
            self.robot.io.score_display_number(self.behavior.score)
            self.release_time = time.time()
        if self.release_time is not None and time.time() - self.release_time >= 0.5:
            return StateDropInScale

    def deinit(self):
        self.robot.io.armothy.close_valve()


class StateGoToChaosZone(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.home()
        if self.behavior.color == Color.YELLOW:
            chaos = self.robot.table.yellow_chaos_zone
        else:
            chaos = self.robot.table.purple_chaos_zone
        c_to_r = self.robot.locomotion.current_pose - chaos.center
        c_to_r_angle = math.atan2(c_to_r.y, c_to_r.x)
        approach_point = Point(chaos.center.x + (chaos.radius + 100) * math.cos(c_to_r_angle),
                               chaos.center.y + (chaos.radius + 100) * math.sin(c_to_r_angle))
        self.robot.locomotion.follow_trajectory([(approach_point.x, approach_point.y,
                                                  center_radians(math.pi + c_to_r_angle))])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateEmptyChaosZone

    def deinit(self):
        self.robot.io.armothy.rotate_y_axis(0)
        self.robot.io.armothy.rotate_z_axis(0)
        self.robot.io.armothy.translate_z_axis(0)


class StateEmptyChaosZone(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_z_axis(0)
        self.robot.io.armothy.rotate_y_axis(0)
        self.robot.io.armothy.home()
        if self.behavior.color == Color.YELLOW:
            self.chaos = self.robot.table.yellow_chaos_zone  # type: ChaosZone
        else:
            self.chaos = self.robot.table.purple_chaos_zone  # type: ChaosZone
        self.start_time = time.time()
        self.storing_side = None
        self.grasping_atom = None
        self.is_storing = False
        self.is_homing = False
        self.search_state = 0

    def test(self):
        if time.time() - self.start_time >= 15 or self.chaos.atoms_remaining() == 0:
            return StateGoToScale

        if not self.robot.locomotion.relative_command_finished:
            print("Moving...")
            return

        if self.is_storing:
            if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
                self.is_storing = False
                self.robot.io.armothy.translate_z_axis(0)
                self.is_homing = True
                if self.grasping_atom is None:
                    print("[FSMMatch] Warning: Removing unknown atom from chaos zone, taking one at random in the zone")
                    self.grasping_atom = self.chaos.random_atom()
                    if self.grasping_atom is None:
                        print("[FSMMatch] Warning: Grasped an atom from an empty Chaos Zone... Insering a new atom")
                        self.grasping_atom = Atom(self.robot.locomotion.x, self.robot.locomotion.y, None, None)
                self.robot.storages[self.storing_side].add(self.grasping_atom)
                self.chaos.remove_atom(self.grasping_atom)
                self.grasping_atom = None
            elif self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.ERROR:
                self.is_storing = False
                self.robot.io.armothy.translate_z_axis(0)
                self.is_homing = True
            else:
                return
        if self.is_homing:
            if self.robot.io.armothy.prismatic_z_axis < 2:
                self.is_homing = False

        closest_puck = self.robot.io.jevois.closest_puck_to_robot
        if closest_puck is not None:
            d_to_center = 450 - closest_puck[0]
            print("d_to_center : ", d_to_center)
            if abs(d_to_center) > 30:
                self.robot.locomotion.turn(math.radians(d_to_center / 8))
            else:
                d_to_grasp = 425 - closest_puck[1]
                print("d_to_grasp : ", d_to_grasp)
                if abs(d_to_grasp) > 20:
                    self.robot.locomotion.go_straight(d_to_grasp / 2.2)
                else:
                    self.grasping_atom = self.chaos.get_atom_in_from_color(Atom.Color(closest_puck[2]))
                    if self.grasping_atom is None:
                        print("[FSMMatch] Could not find any atom with color: {} in {} chaos zone".format(closest_puck[2], self.behavior.color))
                    if closest_puck[2] == "red":
                        if self.behavior.color == Color.YELLOW:
                            self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
                            self.storing_side = AtomStorage.Side.RIGHT
                        else:
                            self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
                            self.storing_side = AtomStorage.Side.LEFT
                    else:
                        if self.behavior.color == Color.YELLOW:
                            self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_take_height(), armothy.eStack.LEFT_STACK)
                            self.storing_side = AtomStorage.Side.LEFT
                        else:
                            self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_take_height(), armothy.eStack.RIGHT_STACK)
                            self.storing_side = AtomStorage.Side.RIGHT
                    self.is_storing = True
        elif not self.is_homing:
            if self.search_state == 0:
                self.robot.locomotion.turn(center_radians(math.atan2(self.chaos.center.y - self.robot.locomotion.y,
                                                                     self.chaos.center.x - self.robot.locomotion.x)
                                                          - self.robot.locomotion.theta))
                self.search_state += 1
            elif self.search_state == 1:
                self.robot.locomotion.turn(math.pi/6)
                self.search_state += 1
            elif self.search_state == 2:
                self.robot.locomotion.turn(-math.pi/3)
                self.search_state += 1
            elif self.search_state == 3:
                self.robot.locomotion.turn(math.pi/6)
                self.search_state += 1
            elif self.search_state == 4:
                self.robot.locomotion.go_straight(50)
                self.search_state = 0

    def deinit(self):
        pass


class StateGoToScale(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(1278, 700, -math.pi/2)])
        else:
            self.robot.locomotion.follow_trajectory([(1692, 700, -math.pi/2)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateEngageScale

    def deinit(self):
        pass


class StateEngageScale(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(150)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateDropInScale

    def deinit(self):
        pass


class StateDropInScale(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.put_in_scale(self.robot.left_storage.armothy_drop_height(),
                                           armothy.eStack.LEFT_STACK,
                                           100 - (-1) ** len(self.robot.left_storage.atoms) * 50, 0)
        else:
            self.robot.io.armothy.put_in_scale(self.robot.right_storage.armothy_drop_height(),
                                               armothy.eStack.RIGHT_STACK,
                                               -100 + (-1) ** len(self.robot.right_storage.atoms) * 50, 0)

    def test(self):
        if self.behavior.color == Color.YELLOW:
            if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
                dropped = self.robot.left_storage.top()
                if dropped.color is not None:
                    self.behavior.score += ScoreInScale[dropped.color]
                else:
                    pass  # Dunno...
                self.robot.left_storage.pop()
                if not self.robot.left_storage.is_empty:
                    self.robot.io.armothy.put_in_scale(self.robot.left_storage.armothy_drop_height(),
                                                               armothy.eStack.LEFT_STACK,
                                                               100 - (-1)**len(self.robot.left_storage.atoms) * 50, 0)
                else:
                    return StateEnd
        else:
            if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
                dropped = self.robot.right_storage.top()
                if dropped.color is not None:
                    self.behavior.score += ScoreInScale[dropped.color]
                else:
                    pass  # Dunno...
                self.robot.right_storage.pop()
                if not self.robot.right_storage.is_empty:
                    self.robot.io.armothy.put_in_scale(self.robot.right_storage.armothy_drop_height(),
                                                       armothy.eStack.RIGHT_STACK,
                                                       -100 + (-1) ** len(self.robot.right_storage.atoms) * 50, 0)
                else:
                    return StateEnd

    def deinit(self):
        pass


#
# class StatePushGreenPeriodic(FSMState):
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         if self.behavior.color == Color.YELLOW:
#             self.robot.locomotion.go_to_orient(250, 1250, math.pi)
#         else:
#             self.robot.locomotion.go_to_orient(2750, 1250, 0)
#
#     def test(self):
#         if self.robot.locomotion.trajectory_finished:
#             return StateRetractGreen
#
#     def deinit(self):
#         self.behavior.score += 1
#         self.robot.io.score_display_number(self.behavior.score)
#
#
# class StateRetractGreen(FSMState):
#     # Todo: Allows the robot to go backward in position control
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         self.start_time = time.time()
#         self.robot.locomotion.set_direct_speed(-100, 0, 0)
#
#     def test(self):
#         if time.time() - self.start_time >= 2.0:
#             return StateEnd
#
#     def deinit(self):
#         pass
#
#
# class StateFrontBluePeriodic(FSMState):
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         if self.behavior.color == Color.YELLOW:
#             self.robot.locomotion.go_to_orient(600, 700, 3*math.pi/4)
#         else:
#             self.robot.locomotion.go_to_orient(2400, 700, math.pi/4)
#
#     def test(self):
#         if self.robot.locomotion.trajectory_finished:
#             return StatePushBluePeriodic
#
#     def deinit(self):
#         pass
#
#
# class StatePushBluePeriodic(FSMState):
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         if self.behavior.color == Color.YELLOW:
#             self.robot.locomotion.go_to_orient(300, 1150, 3*math.pi/4)
#         else:
#             self.robot.locomotion.go_to_orient(2700, 1150, math.pi/4)
#
#     def test(self):
#         if self.robot.locomotion.trajectory_finished:
#             return StateAlignRamp
#
#     def deinit(self):
#         self.behavior.score += 1
#         self.robot.io.score_display_number(self.behavior.score)
#
#
# class StateAlignRamp(FSMState):
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         if self.behavior.color == Color.YELLOW:
#             self.robot.locomotion.go_to_orient(600, 200, 0)
#         else:
#             self.robot.locomotion.go_to_orient(2400, 200, math.pi)
#
#     def test(self):
#         if self.robot.locomotion.trajectory_finished:
#             return StatePushInScale
#
#     def deinit(self):
#         pass
#
#
# class StatePushInScale(FSMState):
#     def __init__(self, behavior):
#         super().__init__(behavior)
#         self.start_time = time.time()
#         self.robot.locomotion.set_direct_speed(10, 0, 0)
#
#     def test(self):
#         if time.time() - self.start_time >= 5:
#             return StateEnd
#
#     def deinit(self):
#         self.behavior.score += 8
#         self.robot.io.score_display_number(self.behavior.score)


class StateEnd(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.set_direct_speed(0, 0, 0)
        self.robot.io.armothy.close_valve()
        self.robot.io.armothy.stop_pump()

    def test(self):
        pass

    def deinit(self):
        pass
