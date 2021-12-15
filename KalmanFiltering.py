from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
import numpy as np


class KalmanFiltering():

    def __init__(self, T) -> None:
        self.f = KalmanFilter(dim_x=3, dim_z=1)

        # set initial state position, speed and acceleration
        self.f.x = np.array([0., 0., 0.])

        # define state transition matrix
        self.f.F = np.array([[1., T, 0.5*T*T],
                             [0., 1., T],
                             [0., 0., 1]])

        # define mesure matrix
        self.f.H = np.array([[1., 0., 0.]])  # z = position

        # define covariance matrix (dimx x dimx)
        self.f.P = np.array([[1.,    0., 0.],
                             [0., 1., 0.],
                             [0., 0., 1.]])

        # define mesarument noise matrix
        self.f.R = np.array([[0.03]])

        self.f.Q = Q_discrete_white_noise(dim=3, dt=T, var=50)

    def getEstimate(self, z):
        self.f.predict()
        self.f.update(z)
        # print(self.f.P_post)
        return self.f.x
