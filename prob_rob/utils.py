import math
import random


def normalize_angle(phi):
    return phi - 2*math.pi * math.floor((phi + math.pi) / (2*math.pi))


def sample_normal_distribution(b):
    return (b/6.0)*sum([random.uniform(-1, 1) for i in range(0, 12)])


def sample_triangular_distribution(b):
    return b * random.uniform(-1, 1) * random.uniform(-1, 1)