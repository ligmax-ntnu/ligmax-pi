from enum import Enum
import numpy as np

# The frontend needs the same code
class Enviroment:
    def __init__(self, upstream_direction: np.ndarray):
        self.upstream_direction = upstream_direction



# The frontend needs the same code
class ObstacleType(Enum):
    UNKNOWN = 0

    RED = 1
    GREEN = 2
    NORTH = 3
    SOUTH = 4
    WEST = 5
    EAST = 6

    BOAT = 7
    LAND = 8
    DOCKING_CENTER = 9

