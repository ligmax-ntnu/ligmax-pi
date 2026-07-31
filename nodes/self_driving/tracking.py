import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
from .pixhalwk import Boat
from .obsticales import ObstacleType

YOLO_CONFIDENCE_PUNISHMENT = 0.5 # multiply colo confidence by this amount to fight overfitting



class Track:
    def __init__(self, track_id: int, pos: np.ndarray, confidence = 0, type: ObstacleType = ObstacleType.UNKNOWN):
        self.track_id = track_id
        self.pos = pos
        self.type = type
        self.confidence = confidence

        self.hits = 1
        self.alpha = 0.3 # the amount a new mesurment will affect the position of the track.

        self.avoid_radius = 1 # keep a distance of 1 meter  

        if type == ObstacleType.BOAT:
            self.positions = [pos]
            self.alpha = 0.5
            self.avoid_radius = 3

    def update(self, mesurment: np.ndarray, confidence: float = 0.0):
        """
        A new mesurment has been received for this track, update the position and type of the track
        """
        self.pos = mesurment * confidence*self.alpha + (1 - confidence*self.alpha) * self.pos

        if self.type == ObstacleType.BOAT:
            self.positions.append(self.pos)

        self.hits += 1
        self.confidence = 1 - (1-confidence)*(1-self.confidence)

    def age(self):
        """ age is called each time update_tracks is called, most likely about 30~40 times a second"""
        self.confidence -= 0.01 



class Tracker:
    def __init__(self, boat: Boat, base_threshold: float = 1.0, uncertainty_factor: float = 0.15):
        self.tracks = []
        self.next_track_id = 0
        self.base_threshold = base_threshold
        self.uncertainty_factor = uncertainty_factor

        self.boat = boat

    def _translate_position(self, angle: float, distance: float, back_lidar: bool = False) -> np.ndarray:
        """
       Translate a position to not be relative to the boat
       angle is angle from the boat in radians, distance is distance from the tip of the boat in meters
        """
        sensor_offset = -0.5 if back_lidar else 0.5 
        relative_pos = np.array([distance * np.cos(angle), distance * np.sin(angle)])
        abs_pos = self.boat.position + relative_pos + self.boat.heading * sensor_offset
        return abs_pos

    def _remove_dead_tracks(self):
        self.tracks = [track for track in self.tracks if track.confidence >= 0.2]

    def _age_tracks(self):
        for track in self.tracks:
            track.age()
             
        self._remove_dead_tracks()

    def update_tracks(self, detections_positions: np.ndarray, detection_types: list, confidences: list):

        if len(detections_positions) == 0:
            self._age_tracks()
            return
        
        # Check if first loop
        if not self.tracks:
            for pos, type, confidence in zip(detections_positions, detection_types, confidences):
                self.tracks.append(Track(self.next_track_id, pos, confidence, type))
                self.next_track_id += 1
            return


        # get current (this might be slow)
        track_positions = np.array([t.position for t in self.tracks]) 

        # Calculate distance matrix between all tracks and all new detections
        dist_matrix = cdist(track_positions, detections_positions)

        # make arrays of the enum type values
        track_types = np.array([t.type.value for t in self.tracks], dtype=np.int8)
        det_types = np.array([t.value for t in detection_types], dtype = np.int8)

        # make columns and rows
        t_col = track_types[:, np.newaxis]
        d_row = det_types[np.newaxis, :]

        # make a matrix mask
        types_mismatch = (t_col != d_row)
        dist_matrix[types_mismatch] = 1e6 # set to an extreamly large number so it cant change live

        # 3. Hungarian Algorithm for optimal assignment
        track_indices, det_indices = linear_sum_assignment(dist_matrix)

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections_positions)))

        for track_idx, det_idx in zip(track_indices, det_indices):
            dist_from_boat = np.linalg.norm(detections_positions[det_idx] - self.boat.position)

            # Calculate the dynamic threshold based on true distance
            dynamic_threshold = self.base_threshold + (self.uncertainty_factor * dist_from_boat)

            match_distance = dist_matrix[track_idx, det_idx]

            if match_distance < dynamic_threshold:
                self.tracks[track_idx].update(detections_positions[det_idx], confidence = confidences[det_idx])
                unmatched_tracks.discard(track_idx)
                unmatched_detections.discard(det_idx)

        # TODO make unnamched new detections
        for track_idx in unmatched_tracks:
            self.tracks[track_idx].age()

        self._remove_dead_tracks()

        for det_idx in unmatched_detections:
            self.tracks.append(Track(self.next_track_id, detections_positions[det_idx], confidences[det_idx], detection_types[det_idx]))

        

