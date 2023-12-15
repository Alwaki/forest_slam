from matplotlib import pyplot as plt
import numpy as np
import copy

x = -np.array([2.395293598881103, 0.011239720827659928, 0.6671715837323203, 3.764595043891987, 1.7425962354592324, 4.783548898472377, 1.118022360442344, 3.075665630074666, 5.461709461145074, 6.152887335830536, 4.430322155122029, 6.793585509374472, -0.4556787612302209, 9.62962751487198, 7.696799309269871, 8.209566408659835, 11.037620737895637, 12.350321233475258, 11.988634158747805, 11.600225346481418, 9.274487992337063, 13.182485104974088, 16.689356473305722, 15.901381546716863, 13.604604784956027, 17.44521082834557, 19.186422628410433, 17.083057418784787, 19.982126498744588, 8.919995574016983, 14.28222052569361, 12.774228264724847, 17.85947283961247, 10.525990828608244, 7.3379189787904435, 8.566568738924353])
y = np.array([2.229795992028524, 3.8496903742884725, 4.6526291900662455, 1.699897255259593, 5.775039082743377, 2.678223439376818, 7.604469497632431, 6.7674541548822145, 8.017891623260013, 4.245550295377092, 8.605539372572952, 6.408326033566166, 3.084595124528592, 5.1080856372222785, 7.176260232353098, 11.258667766383581, 10.249860421756004, 9.064812475911854, 10.75976343809322, 12.30038062306376, 11.87763098270546, 14.569600236406323, 13.821853953000764, 16.1801363314543, 9.897553702473393, 16.911360491045492, 14.925490904090294, 19.02575449420789, 17.479295201925297, 16.551874367559225, 20.615179081163735, 21.466180160459416, 14.212627204316227, 20.021070965552962, 15.518116373956728, 13.447502989876464])

data = np.loadtxt("/home/alwaki/catkin_ws/src/forest_slam/python/data.txt", delimiter='\t', dtype=float)

def find_RMSE(x_const, y_const, data_const, x0, y0, theta):
      x_temp = copy.copy(x_const)
      y_temp = copy.copy(y_const)
      data_temp = copy.copy(data_const)

      for i in range(len(x_temp)):
            x1 = (x[i]*np.cos(theta) - y[i]*np.sin(theta)) + x0
            y1 = (y[i]*np.cos(theta) + x[i]*np.sin(theta)) + y0

            x_temp[i] = x1
            y_temp[i] = y1

      distances = []
      for i in range(len(x_temp)):
            max_dist = 10000
            max_ind = 0
            for j in range(len(data_temp)):
                  dist = np.sqrt(pow(data_temp[j, 0] - x_temp[i], 2) + pow(data_temp[j,1] - y_temp[i], 2))
                  if dist < max_dist:
                        max_dist = dist
                        max_ind = j
            distances.append(max_dist)
      RMSE = np.sqrt(np.mean(pow(np.array(distances), 2)))
      return RMSE

min_error = 1000
min_x = 0
min_y = 0
min_theta = 0

for i in np.linspace(364.3, 364.5, 2):
      for j in np.linspace(425.9, 426.1, 2):
            for k in np.linspace(-0.9, -1.1, 10):
                  error = find_RMSE(x, y, data, i, j, k)
                  if error < min_error:
                        min_error = error
                        min_x = i
                        min_y = j
                        min_theta = k
                        print("RMSE: " + str(error) + " X0: " + str(i) + " Y0: " + str(j) + " Theta: " + str(k))

for i in range(len(x)):
            x1 = (x[i]*np.cos(min_theta) - y[i]*np.sin(min_theta)) + min_x
            y1 = (y[i]*np.cos(min_theta) + x[i]*np.sin(min_theta)) + min_y

            x[i] = x1
            y[i] = y1

plt.plot(data[:,0], data[:,1], "o", markerfacecolor="k", markeredgecolor="k",markersize=4,)
for i, _ in enumerate(data):
        plt.annotate(str(i+1), (data[i, 0], data[i, 1]), fontsize=12)

        
plt.scatter(x,y, color='blue')
plt.scatter(min_x,min_y, color='red')
plt.show()

