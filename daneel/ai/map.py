import yaml


class Map:
    def __init__(self, robot, obstacles_path, obstacle_lidar_mask_path):
        self.robot = robot
        self.lidar_table_bb = None  #   type: BoundingBox
        self.lidar_static_obstacles_bb = []  # type: list[BoundingBox]
        self.static_obstacles = []
        self.load_lidar_static_obstacle(obstacle_lidar_mask_path)
        self.load_obstacles(obstacles_path)

    def load_obstacles(self, obstacles_path):
        self.static_obstacles = []
        with open(obstacles_path, "r") as f:
            obstacles = yaml.load(f)
        for obstacle in obstacles['obstacles']:
            for t, attributes in obstacle.items():
                if t == 'circle':
                    circle = Circle(self.robot, int(attributes['center']['x']), int(attributes['center']['y']),
                                    int(attributes['radius']))
                    self.static_obstacles.append(circle)
                elif t == 'polygon':
                    pts = []
                    for pt in attributes['points']:
                        pts.append((int(pt['x']), int(pt['y'])))
                    polygon = Polygon(self.robot, pts)
                    self.static_obstacles.append(polygon)

    def load_lidar_static_obstacle(self, obstacle_lidar_mask_path):
        with open(obstacle_lidar_mask_path) as f:
            try:
                lidar_obstacles_dict = yaml.load(f)
            except yaml.YAMLError as exc:
                lidar_obstacles_dict = None
                print(exc)

        if lidar_obstacles_dict is not None:
            table = lidar_obstacles_dict['mask']['table']
            self.lidar_table_bb = BoundingBox(self.robot, table['x_start'],
                                              table['y_start'], table['x_stop'], table['y_stop'])
            obstacles = lidar_obstacles_dict['mask']['static_obstacles']
            if obstacles is not None:
                for o in obstacles:
                    x1 = int(o['x_start'])
                    y1 = int(o['y_start'])
                    x2 = int(o['x_stop'])
                    y2 = int(o['y_stop'])
                    self.lidar_static_obstacles_bb.append(BoundingBox(self.robot, x1, y1, x2, y2))


class Obstacle:
    _ID = 0

    def __init__(self, robot):
        self.robot = robot
        self.id = Obstacle._ID
        Obstacle._ID += 1

    def contains(self, x, y):
        raise NotImplementedError()

    def serialize(self):
        raise NotImplementedError()


class BoundingBox(Obstacle):
    def __init__(self, robot, x1, y1, x2, y2):
        super().__init__(robot)
        self.min_x = min((x1, x2))
        self.max_x = max((x1, x2))
        self.min_y = min((y1, y2))
        self.max_y = max((y1, y2))

    def contains(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def serialize(self):

        return "id : {} type : POLYGON points : {},{};{},{};{},{};{},{}".format(self.id, self.min_x, self.min_y,
                                                                                self.min_x, self.max_y,
                                                                                self.max_x, self.max_y,
                                                                                self.max_x, self.min_y)


class Polygon(Obstacle):
    def __init__(self, robot, points):
        super().__init__(robot)
        self.points = points  # Must be like [(x0, y0), (x1, y1), ..., (xn, yn)]

    def contains(self, x, y):
        raise NotImplementedError()

    def serialize(self):
        points = ""
        for pt in self.points:
            points += "{},{};".format(pt[0], pt[1])
        return "id : {} type : POLYGON points : {}".format(self.id, points[:-1])


class Circle(Obstacle):
    def __init__(self, robot, xc, yc, radius):
        super().__init__(robot)
        self.center = (xc, yc)
        self.radius = radius

    def contains(self, x, y):
        return (x - self.center[0])**2 + (y - self.center[1])**2 < self.radius ** 2

    def serialize(self):
        return "id : {} type : CIRCLE center : {},{} radius : {}".format(self.id, self.center[0], self.center[1],
                                                                         self.radius)
