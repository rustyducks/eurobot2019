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
        print("[Atom Storage] Adding atom : ", atom.color)
        self.atoms.append(atom)

    def pop(self):
        print("[Atom Storage] Popping atom : ", self.atoms[-1].color)
        self.atoms.pop()

    def top(self):
        if not self.is_empty:
            return self.atoms[-1]
        return None

    def armothy_take_height(self):
        return 75 - len(self.atoms) * 25

    def armothy_drop_height(self):
        return 100 - len(self.atoms) * 25
