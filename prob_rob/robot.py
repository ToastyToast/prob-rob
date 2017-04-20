import numpy as np


class BaseRobot:
    def __init__(self, x, y, theta, motion_model=None):
        self._pose = np.matrix([x, y, theta]).T
        self._motion = motion_model

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, new_pose):
        self._pose = np.copy(new_pose)

    @pose.deleter
    def pose(self):
        self._pose = None
        del self._pose

    @property
    def motion(self):
        return self._motion

    @motion.setter
    def motion(self, motion_model):
        self._motion = motion_model

    @motion.deleter
    def motion(self):
        del self._motion

    def motion_command(self, command):
        self._pose = self._motion.command(command)
