# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("/home/zhang20/Project/DCAITI_TUB_WS22/IMU_GPS_FUSION_2023/src/result2.txt")


datay_f = data1[:, 7]
datay_o = data1[:, 10]
datax = list(range(1, len(datay_f)+1))
plt.figure("GPS_lon_diff")
plt.title('GPS_lon_diff',verticalalignment='bottom')
plt.ylabel('Longitude')
plt.xlabel('timeline')
plt.scatter(datax, datay_o, s=0.1, c='blue',label='original')
plt.scatter(datax, datay_f, s=0.1, c='red',label='fusion')
plt.legend()
plt.show()