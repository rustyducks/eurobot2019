from enum import Enum


class AtomStorage:
    class Side(Enum):
        LEFT = 0
        RIGHT = 1
    MAX_SIZE = 4

    def __init__(self, robot, side):
        self.robot = robot
        self.side = side
        self.atoms = []  # [0] is bottom of the stack

    @property
    def is_full(self):
        return len(self.atoms) >= self.MAX_SIZE

    @property
    def is_empty(self):
        return len(self.atoms) == 0

    def add(self, atom):
        self.atoms.append(atom)

    def pop(self):
        self.atoms.pop()

    def armothy_height(self):
        return 75 - len(self.atoms) * 25
