import numpy as np
from matplotlib import pyplot as plt

# Federal Test Procedure
ftp = np.loadtxt('uddscol_.txt', dtype=int)
time_s_ftp = ftp[:, 0]
speed_mph_ftp = ftp[:, 1]
speed_kph_ftp = speed_mph_ftp * 1.6
speed_mps_ftp = ftp[:, 1]*0.44704
accel_mpsps_ftp = np.diff(speed_mps_ftp, prepend=0)
distance_m_ftp = np.cumsum(speed_mps_ftp)
distance_mi_ftp = distance_m_ftp * 0.000621371

# Highway Fuel Economy Test
hwy = np.loadtxt('hwycol.txt', dtype=int)
time_s_hwy = hwy[:, 0]
speed_mph_hwy = hwy[:, 1]
speed_kph_hwy = speed_mph_hwy * 1.6
speed_mps_hwy = hwy[:, 1]*0.44704
accel_mpsps_hwy = np.diff(speed_mps_hwy, prepend=0)
distance_m_hwy = np.cumsum(speed_mps_hwy)
distance_mi_hwy = distance_m_hwy * 0.000621371

# City driving conditions
udd = np.loadtxt('uddscol.txt', dtype=int)
time_s_udd = udd[:, 0]
speed_mph_udd = udd[:, 1]
speed_kph_udd = speed_mph_udd * 1.6
speed_mps_udd = udd[:, 1]*0.44704
accel_mpsps_udd = np.diff(speed_mps_udd, prepend=0)
distance_m_udd = np.cumsum(speed_mps_udd)
distance_mi_udd = distance_m_udd * 0.000621371


fig, ax = plt.subplots(figsize=(20, 10))
V1, = ax.plot(time_s_ftp, speed_kph_ftp, 'red')
V2, = ax.plot(time_s_hwy, speed_kph_hwy, 'blue')
V3, = ax.plot(time_s_udd, speed_kph_udd, 'green')
ax.legend((V2, V1, V3), ('Federal Test Procedure', 'Highway Fuel Economy Test',
          'City driving conditions'), loc='upper right', shadow=True)
ax.set_xlabel("Time (sec)")
ax.set_ylabel("Speed (kph)")
ax.set_title('Velocity (kph) vs Time (sec)')
plt.grid()
plt.show()


fig, ax = plt.subplots(figsize=(20, 10))
A1, = ax.plot(time_s_ftp, accel_mpsps_ftp, 'red')
A2, = ax.plot(time_s_hwy, accel_mpsps_hwy, 'blue')
A3, = ax.plot(time_s_udd, accel_mpsps_udd, 'green')
ax.legend((A2, A1, A3), ('Federal Test Procedure', 'Highway Fuel Economy Test',
          'City driving conditions'), loc='upper right', shadow=True)
ax.set_xlabel("Time (sec)")
ax.set_ylabel("Accelaration (mps)")
ax.set_title('Acceleration (g’s) vs Time (sec)')
plt.grid()
plt.show()


fig, ax = plt.subplots(figsize=(20, 10))
D1, = ax.plot(distance_mi_ftp, speed_mph_ftp, 'red')
D2, = ax.plot(distance_mi_hwy, speed_mph_hwy, 'blue')
D3, = ax.plot(distance_mi_udd, speed_mph_udd, 'green')
ax.legend((D2, D1, D3), ('Federal Test Procedure', 'Highway Fuel Economy Test',
          'City driving conditions'), loc='upper right', shadow=True)
ax.set_xlabel("Time (sec)")
ax.set_ylabel("Distance (mi)")
ax.set_title('Velocity (mph) vs Distance (mi)')
plt.grid()
plt.show()
