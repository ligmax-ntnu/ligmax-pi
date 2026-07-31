import numpy as np

class Boat():
    def __init__(self, original_gps_position: np.ndarray, velocity: np.ndarray, heading: np.ndarray):
        self.position = np.array([0,0])
        self.velocity = velocity
        self.heading = heading

        self.original_gps_position = original_gps_position

    def update(self, position: np.ndarray, velocity: np.ndarray, heading: np.ndarray):
        self.position = position
        self.velocity = velocity
        self.heading = heading