from cmath import sqrt
import math
import numpy as np
from matplotlib import pyplot as plt
import math


# Efficiency map data points for IPM-synRM rear motor
RM_Speed_rpm=np.array([200,100,500,12000,3000,1000,2000,6000,10000,4000,2000,10000,8700,2000,2000,4000,8000,1000,6000,4000,4000,6000,5000,3000])
RM_Torque_Nm=np.array([200,230,25,5,230,50,150,100,5,175,125,25,10,100,50,125,25,10,75,160,50,50,50,50])
RM_efficiency=np.array([63,63,90,93,93,94,94,94,94,95,95,95,95,95,96,96,96,96,96,96,97,97,97,97])

plt.tricontourf(list(RM_Speed_rpm),list(RM_Torque_Nm),list(RM_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for IPM-synRM')
plt.show()

# Efficiency map data points for AC Induction front motor
AC_Speed_rpm=np.array([250,1750,500,1500,1000,750,600,1750,750,1750,1500,1000,1000,1000,1750,1500,1250,1500,1750,2500,4750,400,100,100,4000,350,2750,2500,2500,500,250,250,750,1000,750,500])
AC_Torque_Nm=np.array([250,570,63,500,380,200,126,450,189.9,380,430,316,250,63,316,250,63,63,60,60,50,50,60,400,70,80,150,220,70,500,200,570,570,500,430,400])
AC_efficiency=np.array([70,82,85,85,85,86,87,87,89,89,89,89,90,91,91,91,92,92,93,94,92,93,20,50,92,92,92,92,94,72,72,50,72,78,80,87])

plt.tricontourf(list(AC_Speed_rpm),list(AC_Torque_Nm),list(AC_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for AC Induction Motor')
plt.show()

# UDDS Drive Cycle
udds = np.loadtxt('uddscol_.txt', dtype=int)               
udds_time_s = udds[:,0]
udds_speed_mph = udds[:,1]
udds_speed_ms=udds_speed_mph*.44704
udds_accel_ms2=np.diff(udds_speed_ms,prepend=0)

# Vehicle parameters
empty_vehicle_weight_kg=2232

# Wheel parameters with 18" diameter
wheel_radius_m=.2286

# angular velocity of wheel
angular_speed_rads=(udds_speed_ms)/wheel_radius_m
angular_speed_wheel_rpm=angular_speed_rads*(60/2*math.pi)

# F=ma
net_force=empty_vehicle_weight_kg*udds_accel_ms2
force_per_wheel=net_force/4

# torque per wheel in Nm
torque_wheel=force_per_wheel*wheel_radius_m

# motor angular velocity from axle 9:1 ratio, applies to both motors
gear_ratio=9
angular_speed_rpm_motor=angular_speed_wheel_rpm*gear_ratio
torque_motor_Nm=torque_wheel/gear_ratio
power_motor_W=angular_speed_rpm_motor*torque_motor_Nm

# Battery Parameters
battery_capacity_kWh=82
usable_battery_capacity_kWh=75
SOC_full=1.0
SOC_min=0.1

# Plot SOC

Pb = power_motor_W
vb = 400
Rbi = 0.320
Ctb = (82000/vb)
t0 = 0
SOC =1

#print ("FR : ", FR)
#print ("Pb : ", Pb)

time_list = []
SOC_list = []
Sum_SOC_list = []
I0 = ((vb)-((vb**2)-4*Rbi*Pb)**0.5)/(2*Rbi)
#print("I : ", I0)

for t0 in range (8):
    time_list.append(t0)
    t1 = t0+1
    SOC = SOC - (I0/Ctb)*(t1-t0)
#    SOC = (SOC - ((I0/Ctb)*(0.5)*((t1**2)-(t0**2))))

    SOC_Percent = SOC*100
    SOC_list.append(SOC_Percent)
  #   print("SOC: ", SOC)


plt.plot(time_list,SOC_list)
plt.xlabel('Time (h)')
plt.ylabel('Sate of Charge (%)')
plt.grid()
plt.show()
