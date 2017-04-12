import numpy as np


class BaseRobot:
    def __init__(self, x, y, theta, motion_model=None):
        self.pose = np.matrix([x, y, theta]).T
        self.motion = motion_model

    @property
    def pose(self):
        return self.pose

    @pose.setter
    def pose(self, new_pose):
        self.pose = np.copy(new_pose)

    @pose.deleter
    def pose(self):
        self.pose = None
        del self.pose

    @property
    def motion(self):
        return self.motion

    @motion.setter
    def motion(self, motion_model):
        self.motion = motion_model

    @motion.deleter
    def motion(self):
        del self.motion

    def motion_command(self, command):
        self.pose = self.motion.command(command)