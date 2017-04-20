import numpy as np
import math
from .utils import normalize_angle


class MotionModel:
    def __init__(self):
        pass

    def command(self, command):
        raise NotImplementedError


class OdometryMotionModel(MotionModel):
    def __init__(self, pose):
        self._pose = np.copy(pose)

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

    def command(self, command):
        rot1 = command[0]
        trans = command[1]
        rot2 = command[2]

        theta_old = normalize_angle(self.pose.item(2))

        update_vec = np.matrix([
            trans * math.cos(theta_old + rot1),
            trans * math.sin(theta_old + rot1),
            normalize_angle(rot1 + rot2)
        ]).T

        self.pose = self.pose + update_vec

        return self.pose

    @staticmethod
    def command_static(pose, command):
        rot1 = command[0]
        trans = command[1]
        rot2 = command[2]

        theta_old = normalize_angle(pose.item(2))

        update_vec = np.matrix([
            trans * math.cos(theta_old + rot1),
            trans * math.sin(theta_old + rot1),
            normalize_angle(rot1 + rot2)
        ]).T

        pose = pose + update_vec
        pose[2] = normalize_angle(pose.item(2))

        return pose

    @staticmethod
    def sample(pose, command, noise, sample=None):
        if not sample:
            raise ValueError("Provide a sampler")

        rot1 = command[0]
        trans = command[1]
        rot2 = command[2]

        a1, a2, a3, a4 = noise
        theta_old = normalize_angle(pose.item(2))

        rot1_h = rot1 - sample(a1 * rot1 + a2 * trans)
        trans_h = trans - sample(a3 * trans + a4 * (rot1 + rot2))
        rot2_h = rot2 - sample(a1 * rot2 + a2 * trans)

        update_vec = np.matrix([
            trans_h * math.cos(theta_old + rot1_h),
            trans_h * math.sin(theta_old + rot1_h),
            rot1_h + rot2_h
        ]).T

        return pose + update_vec

