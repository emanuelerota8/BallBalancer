
import time
import numpy as np


CLIP_X_MIN =6.25
CLIP_X_MAX =9.25
CLIP_Y_MIN =5
CLIP_Y_MAX = 9

data = np.ones((500,500,3))

for i in range(250):
    time.sleep(1/30)
    data*=2
    data*=2
    data*=2
    data*=2
    