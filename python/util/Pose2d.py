import numpy as np


class Pose2d:
    """
    A class encapsulating a 2 dimensional robot pose.
    """
    def __init__(self, x: float = None, y: float = None, theta: float = None):
        if x is not None:
            self.x = x
        else:
            self.x = 0.0
        if y is not None:
            self.y = y
        else:
            self.y = 0.0
        if theta is not None:
            self.theta = theta
        else:
            self.theta = 0.0

    def relativeTo(self, other: 'Pose2d') -> 'Pose2d':
        """
                Returns the pose of the current object relative to the other pose.
                This involves translating the coordinates and rotating them to the reference frame of the other pose.
                """
        # Translate coordinates
        dx = self.x - other.x
        dy = self.y - other.y

        # Rotate the translation by the inverse of the other pose's orientation
        cos_theta = np.cos(-other.theta)
        sin_theta = np.sin(-other.theta)

        # Apply the rotation matrix to get the relative position in the new coordinate frame
        rel_x = cos_theta * dx - sin_theta * dy
        rel_y = sin_theta * dx + cos_theta * dy

        # The relative orientation is simply the difference in theta
        rel_theta = self.theta - other.theta

        return Pose2d(rel_x, rel_y, rel_theta)