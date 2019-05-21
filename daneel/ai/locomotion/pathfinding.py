import math
import numpy as np

GRAPH_FILE = "data/nav_graph.pbm"
TABLE_HEIGHT = 2000
TABLE_WIDTH = 3000


class PathFinding:
    def __init__(self, robot):
        self.robot = robot

    def find_path(self, start, goal):
        raise NotImplementedError("This is an abstract class, must be implemented before use.")

class Node:
    def __init__(self, value, x, y):
        self.value = value
        self.x = x
        self.y = y
        self.parent = None
        self.H = 0
        self.G = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def heuristic(self, other):
        return self.manhattan_distance(other)

    def distance(self, other):
        return math.sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)

    def cost(self, other):
        return self.distance(other)

    def manhattan_distance(self, other):
        return abs(other.x - self.x) + abs(other.y - self.y)

    def __hash__(self):
        return self.x.__hash__() + self.y.__hash__()

    def __repr__(self):
        return "x:{} y:{} value:{} h:{} g:{}".format(self.x, self.y, self.value, self.H, self.G)

    def __str__(self):
        return "x:{} y:{} value:{} h:{} g:{}".format(self.x, self.y, self.value, self.H, self.G)

    def reset(self):
        self.parent = None
        self.H = 0
        self.G = 0

class ThetaStar(PathFinding):
    def __init__(self, robot):
        super().__init__(robot)
        self.graph = None
        self.graph_table_ratio = 0
        self.load_graph(GRAPH_FILE)
        self.height = len(self.graph[0])
        self.width = len(self.graph)

    def load_graph(self, file):
        with open(file, 'r') as f:
            magic_number = None
            img_size = None
            j = 0
            for line in f:
                if line.startswith('#'):
                    continue

                if line.strip() in ["P1", "P2", "P3", "P4", "P5"]:
                    magic_number = line.strip()
                    continue
                elif magic_number is not None and img_size is None:
                    img_size = list(map(int, line.strip().split()))
                    print("Image size :", img_size)
                    self.graph = np.empty((img_size[0], img_size[1]),
                                          dtype=Node)
                    self.graph_table_ratio = img_size[0] / TABLE_WIDTH
                    print("Graph Table ratio :", self.graph_table_ratio)
                    assert img_size[1] / TABLE_HEIGHT == self.graph_table_ratio
                    continue
                for i, pt in enumerate(line.strip()):
                    value = True if pt == '0' else False
                    self.graph[i,j] = Node(value, i, j)
                j += 1
            # self.graph = list(reversed(self.graph))
            f.close()
        print(self.graph)

    def find_path(self, start, goal):
        print("[Theta*] Resetting graph")
        self.reset_graph()
        print("[Theta*] Searching for a path from {} to {}".format(start, goal))
        opened = set()
        closed = set()
        start_node = self.graph[int(start[0] * self.graph_table_ratio)][int(start[1] * self.graph_table_ratio)]
        if not start_node.value:
            print("[Theta*] Start position in obstacle. Aborting.")
            return
        goal_node = self.graph[int(goal[0] * self.graph_table_ratio)][int(goal[1] * self.graph_table_ratio)]
        if not goal_node.value:
            print("[Theta*] Goal position in obstacle. Aborting.")
            return
        start_node.H = start_node.heuristic(goal_node)
        opened.add(start_node)
        while len(opened) != 0:
            s = min(opened, key=lambda n: n.G + n.H)
            opened.remove(s)
            if s == goal_node:
                path = []
                s_path = s
                while s_path.parent is not None:
                    path.append((s_path.x // self.graph_table_ratio, s_path.y // self.graph_table_ratio))
                    s_path = s_path.parent
                print("[Theta*] Path found from {} to {}. Trajectory length: {}".format(start, goal, len(path)))
                return list(reversed(path))
            closed.add(s)
            for s_2 in self.neighbours(s):
                if s_2 not in closed:
                    if s_2 not in opened:
                        s_2.G = -1
                        s_2.parent = None
                    self.update_node(s, s_2, opened, goal_node)
        print("No Path found")
        return

    def reset_graph(self):
        for c in self.graph:
            for n in c:
                n.reset()

    def update_node(self, s, s_2, opened, goal_node):
        g_old = s_2.G
        self.compute_cost(s, s_2)
        if s_2.G < g_old:
            opened.remove(s_2)
        s_2.H = s_2.heuristic(goal_node)
        opened.add(s_2)

    def compute_cost(self,s, s_2):
        if s.parent is not None:
            if self.line_of_sight(s.parent, s_2):
                if s.parent.G + s_2.cost(s.parent) < s_2.G or s_2.G == -1:
                    s_2.parent = s.parent
                    s_2.G = s.parent.G + s_2.cost(s.parent)
        if s.G + s_2.cost(s) < s_2.G or s_2.G == -1:
            s_2.parent = s
            s_2.G = s.G + s_2.cost(s)

    def neighbours(self, s):
        neighbours = []
        if s.x > 0 and self.graph[s.x - 1][s.y].value:
            neighbours.append(self.graph[s.x - 1][s.y])
        if s.x < self.width - 1 and self.graph[s.x + 1][s.y].value:
            neighbours.append(self.graph[s.x + 1][s.y])
        if s.y > 0 and self.graph[s.x][s.y - 1].value:
            neighbours.append(self.graph[s.x][s.y - 1])
        if s.y < self.height - 1 and self.graph[s.x][s.y + 1].value:
            neighbours.append(self.graph[s.x][s.y + 1])
        return neighbours

    def line_of_sight(self, s, s_2):
        x0 = s.x
        y0 = s.y
        x1 = s_2.x
        y1 = s_2.y
        dy = y1 - y0
        dx = x1 - x0
        f = 0
        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1
        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1
        if dx >= dy:
            while x0 != x1:
                f = f + dy
                if f >= dx:
                    if not self.graph[x0 + (sx - 1) // 2][y0 + (sy - 1) // 2].value:
                        return False
                    y0 = y0 + sy
                    f = f - dx
                if f != 0 and not self.graph[x0 + (sx - 1) // 2][y0 + (sy - 1) // 2].value:
                    return False
                if dy == 0 and not self.graph[x0 + (sx - 1) // 2][y0].value and not self.graph[x0 + (sx - 1) // 2][y0 - 1].value:
                    return False
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx
                if f >= dy:
                    if not self.graph[x0 + (sx - 1)//2][y0 + (sy - 1)//2].value:
                        return False
                    x0 = x0 + sx
                    f = f - dy
                if f != 0 and not self.graph[x0 + (sx - 1)//2][y0 + (sy - 1)//2].value:
                    return False
                if dx == 0 and not self.graph[x0][y0 + (sy - 1)//2].value and not self.graph[x0 - 1][y0 + (sy - 1)//2].value:
                    return False
                y0 = y0 + sy
        return True