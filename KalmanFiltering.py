from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
import numpy as np
import time


class KalmanFiltering():

    def __init__(self, T) -> None:
        self.f = KalmanFilter(dim_x=2, dim_z=1)

        # set initial state position and speed
        self.f.x = np.array([0., 0.])

        # define state transition matrix
        self.f.F = np.array([[1., T],
                             [0., 1.]])

        # define mesure matrix
        self.f.H = np.array([[1., 0.]])  # z = x

        # define covariance matrix (dimx x dimx)
        self.f.P = np.array([[1.,    0.],
                             [0., 1.]])

        # define mesarument noise matrix
        self.f.R = np.array([[0.005]])

        self.f.Q = Q_discrete_white_noise(dim=2, dt=T, var=0.7*10)

    def getEstimate(self, z):
        self.f.predict()
        self.f.update(z)
        return self.f.x


if __name__ == "__main__":
    t = 1/30
    kal = KalmanFiltering(t)
    while True:
        filteredValue = kal.getEstimate(5)
        print(filteredValue)
        time.sleep(1)
