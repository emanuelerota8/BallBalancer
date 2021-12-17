from os import X_OK
import pickle
import matplotlib.pyplot as plt


file = open("x",'rb')
data = pickle.load(file)

x = data[0]
y = data[1]
xK = data[2]
yK = data[3]

plt.plot(x)
#plt.plot(xK)
plt.legend(["x"])
plt.show()