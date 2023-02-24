# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("/home/zhang20/Project/DCAITI_TUB_WS22/IMU_GPS_FUSION_2023/build/bin/dataout.txt")

num1 = data1.shape
num = data1.size
datay_fx = data1[:, 11]
datay_fy = data1[:, 12]
datay_fz = data1[:, 13]

datay_ox = data1[:, 14]
datay_oy = data1[:, 15]
datay_oz = data1[:, 16]
datax = list(range(1, len(datay_fx)+1))
plt.figure(figsize=(12, 9))

plt.subplot(2, 1, 1)
data_corrx = datay_fx - datay_ox
plt.bar(range(len(data_corrx)), data_corrx)
# plt.title('diff_x')
plt.xlabel('Timeline')
plt.ylabel('diff_x')


plt.subplot(2, 1,2)
data_corry = datay_fy - datay_oy
plt.bar(range(len(data_corry)), data_corry)
# plt.title('diff_y')
plt.xlabel('Timeline')
plt.ylabel('diff_y')

# plt.subplot(3, 1,3)
# data_corrz = datay_fz - datay_oz
# plt.bar(range(len(data_corrz)), data_corrz)
# plt.title('diff_z')
# plt.xlabel('Timeline')
# plt.ylabel('z')
# plt.savefig("test.png")

plt.show()



