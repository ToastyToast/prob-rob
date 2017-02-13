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
        mean = np.dot(self.A, self.mean) + np.dot(self.B, control)
        cov = self.A.dot(self.cov).dot(self.A.T) + self.R

        K = cov.dot(self.C.T).dot(
            self.C.dot(cov).dot(self.C.T) + self.Q
        ).T

        self.mean = mean + K.dot(measurement - self.C.dot(mean))
        size = self.cov.shape[0]
        self.cov = (np.eye(size) - K.dot(self.C)).dot(cov)

