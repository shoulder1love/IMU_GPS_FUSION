# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# load data from file
# you can replace this using with open
data1 = np.loadtxt("/home/zhang20/Project/DCAITI_TUB_WS22/IMU_GPS_FUSION_2023/build/bin/dataout.txt")



num=data1.size




datax = data1[:, 11]
datay = data1[:, 12]
dataz = data1[:, 13]

data_gpsx = data1[:, 14]
data_gpsy = data1[:, 15]
data_gpsz = data1[:, 16]

print (datax)
print (datay)
print (dataz)

numx=datax.size
print(numx)
numy=datay.size
print(numy)
numz=dataz.size
print(numz)

# new a figure and set it into 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

# set figure information
ax.set_title("3D_Curve")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")


# fig_gps = plt.figure()
# ax_gps = fig.gca(projection='3d_gps')
# ax_gps.set_title("3D_Curve")
# ax_gps.set_xlabel("x")
# ax_gps.set_ylabel("y")
# ax_gps.set_zlabel("z")


# draw the figure, the color is r = read

figure = ax.plot(datax, datay, dataz, c='r', label = 'fusion')

figure = ax.plot(data_gpsx, data_gpsy, data_gpsz, c='b', label = 'original')



# figure_gps = ax_gps.plot(data_gpsx, data_gpsy, data_gpsz, c='b')

plt.show()
