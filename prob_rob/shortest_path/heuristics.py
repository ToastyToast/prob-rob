from math import fabs, sqrt


def manhattan(x1, y1, x2, y2):
    return fabs(x1 - x2) + fabs(y1 - y2)


def euclidean(x1, y1, x2, y2):
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

