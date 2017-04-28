import numpy as np
import math
from prob_rob.utils import normalize_angle


def odometry_command(pose, command):
    rot1, trans, rot2 = command
    theta_old = normalize_angle(pose.item(2))

    update_vec = np.matrix([
        trans * math.cos(theta_old + rot1),
        trans * math.sin(theta_old + rot1),
        normalize_angle(rot1 + rot2)
    ]).T

    pose = pose + update_vec
    pose[2] = normalize_angle(pose.item(2))

    return np.copy(pose)


def odometry_sample(pose, command, noise, sample=None):
    if not sample:
        raise ValueError("Provide a sampler")

    rot1, trans, rot2 = command

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

    return np.copy(pose + update_vec)

