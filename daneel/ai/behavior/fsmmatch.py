from enum import Enum
import time
import math
from math import pi
import os
import armothy
from table.table import SlotName, Atom
from robot_parts import AtomStorage

from behavior import Behavior

END_MATCH_TIME = 100  # in seconds

WARNING_VOLTAGE_THRESHOLD = 11.5  # if signal or power voltage goes under this value, led will be flashing red.


class Color(Enum):
    YELLOW = "yellow"
    PURPLE = "purple"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.score = 0
        self.state = StatePreStartChecks(self)
        self.shutdown_button_press_time = 0

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
            if __debug__:
                print("[FSMMatch] End match")
            next_state = StateEnd
        else:
            next_state = self.state.test()
        if next_state is not None:
            if __debug__:
                print("[FSMMatch] Leaving {}, entering {}".format(self.state.__class__.__name__, next_state.__name__))
            self.state.deinit()
            self.state = next_state(self)

    def start_match(self):
        if __debug__:
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
                self.robot.io.score_display_fat()

        if self.robot.io.button2_state == self.robot.io.ButtonState.PRESSED:
            return StateColorSelection

    def deinit(self):
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_POWER, self.robot.io.SensorState.STOPPED)
        self.robot.io.change_sensor_read_state(self.robot.io.SensorId.BATTERY_SIGNAL, self.robot.io.SensorState.STOPPED)
        self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
        self.robot.io.score_display_fat()


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
            self.robot.locomotion.reposition_robot(145, 1255, 0)
        else:
            self.robot.locomotion.reposition_robot(2855, 1570, -math.pi)
        self.robot.io.set_led_color(self.robot.io.LedColor.WHITE)


class StatePreMatch(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)

    def test(self):
        if self.robot.io.cord_state == self.behavior.robot.io.CordState.OUT:
            return StateFrontGreenPeriodic

    def deinit(self):
        self.behavior.start_match()
        self.behavior.score = 0
        self.robot.io.score_display_number(self.behavior.score)


class StateFrontGreenPeriodic(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_z_axis(0)
        if self.behavior.color == Color.YELLOW:
            self.robot.locomotion.follow_trajectory([(305, 1250, 0)])
        else:
            pass
            # self.robot.locomotion.follow_trajectory([(2695, 1250, 0)])

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateGetColorFrontGreenPeriodicAtom

    def deinit(self):
        pass


class StateGetColorFrontGreenPeriodicAtom(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.armothy.rotate_z_axis(0)
        self.start_time = time.time()
        self.time_out = 2  # sec

    def test(self):
        seen_color = self.robot.io.jevois.uniq_last_puck_color
        if seen_color is not None:
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
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
                self.store_side = AtomStorage.Side.RIGHT
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
                self.store_side = AtomStorage.Side.LEFT
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color == Atom.Color.RED:
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
                self.store_side = AtomStorage.Side.LEFT
            else:
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
                self.store_side = AtomStorage.Side.RIGHT
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
            self.robot.storages[self.store_side].add(self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom)
            # return StateGoToFrontBluePeriodic
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
        # TODO(guilhem): Add for purple side as well
        self.robot.locomotion.follow_trajectory([(500, 1250, -pi / 2), (500, 1145, -pi / 2)])

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
            if self.behavior.color == Color.YELLOW:
                self.robot.table.slots[SlotName.YELLOW_PERIODIC_BLUE].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green is still there...
                    self.robot.table.slots[SlotName.YELLOW_PERIODIC_GREEN].atom.color = Atom.Color.RED
                return StateTakeSecondAtom
            else:
                self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color = Atom.Color(seen_color)
                if seen_color == "green":
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color = Atom.Color.RED
                    # TODO(guilhem) Check if the periodic green is still there...
                    self.robot.table.slots[SlotName.PURPLE_PERIODIC_GREEN].atom.color = Atom.Color.RED
                return StateTakeSecondAtom
        if time.time() - self.start_time >= self.time_out:
            return StateTakeSecondAtom

        def deinit(self):
            pass

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
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_BLUE].atom.color == Atom.Color.RED:
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
            else:
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
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
        # TODO(guilhem): Add for purple side as well
        self.robot.locomotion.follow_trajectory([(500, 1155, pi / 2), (500, 1345, pi / 2)])

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
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
            else:
                #  If the atom is green or if we don't know, we put it in the left stack to put it on the scale
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
        else:
            if self.robot.table.slots[SlotName.PURPLE_PERIODIC_RED].atom.color == Atom.Color.RED:
                self.store_side = AtomStorage.Side.LEFT
                self.robot.io.armothy.take_and_store(self.robot.left_storage.armothy_height(), armothy.eStack.LEFT_STACK)
            else:
                self.store_side = AtomStorage.Side.RIGHT
                self.robot.io.armothy.take_and_store(self.robot.right_storage.armothy_height(), armothy.eStack.RIGHT_STACK)
        self.start_time = time.time()
        self.timeout = 7  # sec

    def test(self):
        if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
            self.robot.storages[self.store_side].add( self.robot.table.slots[SlotName.YELLOW_PERIODIC_RED].atom)
            return StateDropRediums

        if time.time() - self.start_time >= self.timeout:
            self.robot.io.armothy.home()
            # Do not add the atom to the atom tank
            return StateDropRediums

    def deinit(self):
        pass


class StateDropRediums(FSMState):
    # TODO Purple
    def __init__(self, behavior):
        super().__init__(behavior)
        self.traj_finished = False
        self.atom1_dropped = False
        self.atom2_droping = False
        self.robot.locomotion.go_to_orient(500, 1550, -pi)
        self.start_time = time.time()
        self.drop_timeout = 5
        self.dropped = 0

    def test(self):
        if not self.traj_finished:
            if self.robot.locomotion.trajectory_finished:
                self.traj_finished = True
                if self.behavior.color == Color.YELLOW:
                    self.robot.io.armothy.put_down(self.robot.right_storage.armothy_height(),
                                                   armothy.eStack.RIGHT_STACK, 100)
                    self.start_time = time.time()
        else:
            if self.robot.io.armothy.get_macro_status() == armothy.eMacroStatus.FINISHED:
                self.dropped += 1
                if self.behavior.color == Color.YELLOW:
                    self.robot.right_storage.pop()
                    if not self.robot.right_storage.is_empty:
                        self.robot.io.armothy.put_down(self.robot.right_storage.armothy_height(),
                                                       armothy.eStack.RIGHT_STACK,
                                                       (-1)**len(self.robot.right_storage.atoms) * 100)
                        self.start_time = time.time()
                    else:
                        return StateDisengageRedPeriodic
            elif time.time() - self.start_time >= self.drop_timeout:
                return StateDisengageRedPeriodic

    def deinit(self):
        self.behavior.score += self.dropped * 6  # 6 pts per Red in Red periodic zone


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
            self.robot.locomotion.navigate_to(1710, 1700, math.pi/2)
            self.robot.io.armothy.rotate_z_axis(200)
        else:
            self.robot.locomotion.navigate_to(1290, 1700, math.pi/2)
            self.robot.io.armothy.rotate_z_axis(-200)
        self.robot.io.armothy.translate_z_axis(45)
        self.robot.io.armothy.rotate_y_axis(330)

    def test(self):
        if self.robot.locomotion.trajectory_finished:
            return StateEngageParticleAccelerator

    def deinit(self):
        pass


class StateEngageParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.locomotion.go_straight(50)

    def test(self):
        if self.robot.locomotion.relative_command_finished:
            return StateRollParticleAccelerator


    def deinit(self):
        pass


class StateRollParticleAccelerator(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.YELLOW:
            self.robot.io.armothy.rotate_z_axis(-50)
            self.robot.locomotion.turn(math.radians(7))
        else:
            # TODO set z axis for Purple
            self.robot.io.armothy.rotate_z_axis(200)
        self.start_time = time.time()

    def test(self):
        if time.time() - self.start_time >= 1:
            return StateEnd

    def deinit(self):
        self.behavior.score += 10  # One atom in the accelerator
        self.behavior.score += 10  # Goldenium revealed !
        self.robot.io.score_display_number(self.behavior.score)

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

    def test(self):
        pass

    def deinit(self):
        pass
