from enum import Enum

class Table:
    def __init__(self, robot):
        self.robot = robot
        self.slots = {
            SlotName.YELLOW_PERIODIC_RED: AtomSlot(SlotName.YELLOW_PERIODIC_RED, 500, 1550),
            SlotName.YELLOW_PERIODIC_GREEN: AtomSlot(SlotName.YELLOW_PERIODIC_GREEN, 500, 1250),
            SlotName.YELLOW_PERIODIC_BLUE: AtomSlot(SlotName.YELLOW_PERIODIC_BLUE, 500, 1050),
            SlotName.YELLOW_RAMP: AtomSlot(SlotName.YELLOW_RAMP, 834, 200, Atom.Color.GREEN),
            SlotName.YELLOW_DISPENSER_SMALL: [
                AtomSlot(SlotName.YELLOW_DISPENSER_SMALL, 125, 0, Atom.Color.BLUE, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_SMALL, 225, 0, Atom.Color.GREEN, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_SMALL, 325, 0, Atom.Color.RED, 128.5)
            ],
            SlotName.YELLOW_DISPENSER_LARGE: [
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 500, 1543, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 600, 1543, Atom.Color.GREEN, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 700, 1543, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 800, 1543, Atom.Color.BLUE, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 900, 1543, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.YELLOW_DISPENSER_LARGE, 1000, 1543, Atom.Color.GREEN, 128.5)
            ],
            SlotName.PURPLE_SENSOR: AtomSlot(SlotName.PURPLE_SENSOR, 775, 1950, Atom.Color.WHITE, 190),
            SlotName.PURPLE_ACCELERATOR: AtomSlot(SlotName.PURPLE_ACCELERATOR, 1290, 1950, Atom.Color.BLUE, 190),
            SlotName.YELLOW_ACCELERATOR: AtomSlot(SlotName.YELLOW_ACCELERATOR, 1710, 1950, Atom.Color.BLUE, 190),
            SlotName.YELLOW_SENSOR: AtomSlot(SlotName.YELLOW_SENSOR, 2225, 1950, Atom.Color.WHITE, 190),
            SlotName.PURPLE_DISPENSER_LARGE: [
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2000, 1543, Atom.Color.GREEN, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2100, 1543, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2200, 1543, Atom.Color.BLUE, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2300, 1543, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2400, 1543, Atom.Color.GREEN, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_LARGE, 2500, 1543, Atom.Color.RED, 128.5)
            ],
            SlotName.PURPLE_DISPENSER_SMALL: [
                AtomSlot(SlotName.PURPLE_DISPENSER_SMALL, 2675, 0, Atom.Color.RED, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_SMALL, 2775, 0, Atom.Color.GREEN, 128.5),
                AtomSlot(SlotName.PURPLE_DISPENSER_SMALL, 2875, 0, Atom.Color.BLUE, 128.5)
            ],
            SlotName.PURPLE_RAMP: AtomSlot(SlotName.PURPLE_RAMP, 2166, 200, Atom.Color.GREEN),
            SlotName.PURPLE_PERIODIC_BLUE: AtomSlot(SlotName.PURPLE_PERIODIC_BLUE, 2500, 1050),
            SlotName.PURPLE_PERIODIC_GREEN: AtomSlot(SlotName.PURPLE_PERIODIC_GREEN, 2500, 1250),
            SlotName.PURPLE_PERIODIC_RED: AtomSlot(SlotName.PURPLE_PERIODIC_RED, 2500, 1550)
        }

class SlotName(Enum):
    YELLOW_PERIODIC_RED = 0
    YELLOW_PERIODIC_GREEN = 1
    YELLOW_PERIODIC_BLUE = 2
    YELLOW_RAMP = 3
    YELLOW_DISPENSER_SMALL = 4
    YELLOW_DISPENSER_LARGE = 5
    PURPLE_SENSOR = 6
    PURPLE_ACCELERATOR = 7
    YELLOW_ACCELERATOR = 8
    YELLOW_SENSOR = 9
    PURPLE_DISPENSER_LARGE = 10
    PURPLE_DISPENSER_SMALL = 11
    PURPLE_RAMP = 12
    PURPLE_PERIODIC_BLUE = 13
    PURPLE_PERIODIC_GREEN = 14
    PURPLE_PERIODIC_RED = 15


class ChaosZone:
    def __init__(self):
        self.atoms = []


class Atom:
    class Color(Enum):
        RED = "red"
        GREEN = "green"
        BLUE = "blue"
        WHITE = "white"

    def __init__(self, x, y, color, vertical):
        self.position = (x, y)
        self.carried = False
        self.color = color
        if vertical is None or not vertical:
            self.is_vertical = False
            self.height = 0
        else:
            self.is_vertical = True
            self.height = vertical


class AtomSlot:
    def __init__(self, name, x, y, initial_color=None, height=None):
        self.name = name
        self.height = height
        self.position = (x, y)
        self.atom = Atom(x, y, initial_color, height)
        self.is_in_slot = True


ScoreInScale = {Atom.Color.RED: 8, Atom.Color.GREEN: 10, Atom.Color.BLUE: 12, Atom.Color.WHITE: 24}  # TODO: Check


