import numpy as np


class BaseGaussianFilter:
    def __init__(self, A, B, C, mean, cov, R, Q):
        pass

    def step(self, control, measurement):
        raise NotImplementedError

class KalmanFilter(BaseGaussianFilter):
    def __init__(self, A, B, C, mean, cov, R, Q):
        self.A = A
        self.B = B
        self.C = C
        self.mean = mean
        self.cov = cov
        self.R = R
        self.Q = Q

    def step(self, control, measurement):
        mean = self.A * self.mean + self.B * control
        cov = self.A * self.cov * self.A.T + self.R

        K = cov * self.C.T * (
            self.C * cov * self.C.T + self.Q
        ).T

        self.mean = mean + K * (measurement - self.C * mean)
        size = self.cov.shape[0]
        self.cov = (np.eye(size) - K * self.C) * cov

