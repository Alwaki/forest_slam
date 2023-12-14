from matplotlib import pyplot as plt
import numpy as np

x = -np.array([3.78455901874402, -0.25953457648621825, 1.5529746844457617, 3.3467716224868562, -1.5461044410822158, 2.1557716368619673, 4.671655404097218, 5.3987364342016075, 7.779451170672353, 4.315927710402041, 6.114711259180154, 10.395803019033293, 11.206459676423062, 8.493278858130118, 13.55590995775072, 8.89924694782531, 11.78352949556478, 6.6478499281033345, 5.035013131443131, 2.548748718143389])

y = np.array([1.400754687249016, -2.4117344569697523, -1.2908671962268596, -2.8094663453380773, -2.05222788831035, 1.752424806577019, 3.2256723837233916, 1.0446441613648207, 0.16185936476594223, -5.301576307690434, -3.598776962164851, 0.5602739991649286, -4.05388937557657, -4.816920405305094, -0.23020562176889814, -6.271727968511387, -9.160504339205021, -8.458843371292481, -6.882934981086478, -3.1638239412636127])


x0 = 365.6
y0 = 427.0

data = np.loadtxt("/home/alwaki/catkin_ws/src/forest_slam/python/data.txt", delimiter='\t', dtype=float)
plt.plot(data[:,0], data[:,1], "*", markerfacecolor="k", markeredgecolor="k",markersize=12,)
for i, _ in enumerate(data):
        plt.annotate(str(i+1), (data[i, 0], data[i, 1]), fontsize=12)
theta = -2
for i in range(len(x)):
    x1 = (x[i]*np.cos(theta) - y[i]*np.sin(theta)) + x0
    y1 = (y[i]*np.cos(theta) + x[i]*np.sin(theta)) + y0

    x[i] = x1
    y[i] = y1


plt.scatter(x,y, color='blue')
plt.scatter(x0,y0, color='red')
plt.show()

distances = []
for i in range(len(x)):
      max_dist = 10000
      max_ind = 0
      for j in range(len(data)):
            dist = np.sqrt(pow(data[j, 0] - x[i], 2) + pow(data[j,1] - y[i], 2))
            if dist < max_dist:
                  max_dist = dist
                  max_ind = j
      distances.append(max_dist)
print(distances)
RMSE = np.sqrt(np.mean(pow(np.array(distances), 2)))
print(RMSE)
    
