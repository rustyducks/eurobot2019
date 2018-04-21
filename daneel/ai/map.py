import yaml


class Map:
    def __init__(self, robot, obstacle_lidar_mask_path):
        self.robot = robot
        self.lidar_table_bb = None  #   type: BoundingBox
        self.lidar_static_obstacles_bb = []  # type: list[BoundingBox]
        self.load_lidar_static_obstacle(obstacle_lidar_mask_path)

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

    def is_in(self, x, y):
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
