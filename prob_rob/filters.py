import numpy as np


class BaseFilter:
    def step(self, control, measurement):
        raise NotImplementedError

class KalmanFilter(BaseFilter):
    """Kalman filter

    Parameters
    ----------
    A:
        state transition matrix
    B:
        control matrix
    C:
        observation matrix
    mean:
        Initial state estimate
    cov:
        Initial covariance estimate
    R:
        Error in process
    Q:
        Error in measurements
    """
    def __init__(self, A, B, C, mean, cov, R, Q):
        self.A = A
        self.B = B
        self.C = C
        self.mean = mean
        self.cov = cov
        self.R = R
        self.Q = Q

    def step(self, control, measurement):
        """Perform a kalman filter step

        Parameters
        ----------
        control:
            control matrix
        measurement:
            measurement matrix
        """
        mean = self.A * self.mean + self.B * control
        cov = (self.A * self.cov) * self.A.T + self.R

        K = cov * self.C.T * (
            (self.C * cov) * self.C.T + self.Q
        ).T

        self.mean = mean + K * (measurement - self.C * mean)
        size = self.cov.shape[0]
        self.cov = (np.eye(size) - K * self.C) * cov


class ExtendedKalmanFilter(BaseFilter):
    """Extended Kalman Filter

    Parameters
    ----------
    G:
        Jacobian, which corresponds to A and B matrices in KF
    H:
        Jacobian, which corresponds to C matrix in KF
    mean:
        Initial state estimate
    cov:
        Initial covariance estimate
    R:
        Error in process
    Q:
        Error in measurements
    state_pred: function
        Nonlinear state prediction function
        Args:
            control: control matrix
            state: intial state
        Return value:
            predicted state
    meas_pred: function
        Nonlinear measurement prediction function
        Args:
            state: predicted state
        Return value:
            predicted measurement
    """
    def __init__(self, G, H, mean, cov, R, Q,
                 state_pred=None, meas_pred=None):
        self.G = G
        self.H = H
        self.mean = mean
        self.cov = cov
        self.R = R
        self.Q = Q

        if not state_pred or not meas_pred:
            raise ValueError("Must provide state prediction\
                    and measurement functions")

        self.state_pred = state_pred
        self.meas_pred = meas_pred

    def step(self, control, measurement):
        mean = self.state_pred(control, self.mean)
        cov = (self.G * self.cov) * self.G.T + self.R

        K = cov * self.H.T * (
            (self.H * cov) * self.H.T + self. Q
        ).T

        self.mean = mean + K * (measurement-self.meas_pred(mean))
        size = self.cov.shape[0]
        self.cov = (np.eye(size) - K * self.H) * cov

