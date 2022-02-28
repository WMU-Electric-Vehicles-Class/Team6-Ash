from cmath import sqrt
import math
import numpy as np
from matplotlib import pyplot as plt
import math

# UDDS Drive Cycle
udds = np.loadtxt('uddscol_.txt', dtype=int)               
udds_time_s = udds[:,0]
udds_speed_mph = udds[:,1]
udds_speed_ms=udds_speed_mph*.44704
udds_accel_ms2=np.diff(udds_speed_ms,prepend=0)

# Vehicle parameters
empty_vehicle_weight_kg=2232
g=9.81

#Force due to mass of the vehicel (Fm) in Newton
Fm=empty_vehicle_weight_kg*udds_accel_ms2

#Force due to Rolling Resistance (Frr) in Newton
#Crr = Rolling Resistance Coefficient
Crr=0.015 #Assumed from example probelm
Frr=Crr*empty_vehicle_weight_kg*g

#Aerodynamic force (Fd) in Newton
#Cd=Drag Coefficient
Cd=0.28
Air_Density=1.2
Vehicle_Height_m=1.443
Vehicle_Width_m=1.849
Vehicle_Front_Area_m2=Vehicle_Height_m*Vehicle_Width_m
Fd=0.5*Cd*Air_Density*Vehicle_Front_Area_m2*udds_speed_ms**2

#Total Force (Ft) in Newton
Ft=Fd+Frr+Fm

#Force in each wheels (Fw) in Newton
Fw=Ft/4

# Wheel parameters with 18" diameter
wheel_radius_m=.2286

# torque per wheel in Nm
wheel_torque_Nm=Fw*wheel_radius_m

# angular velocity of wheel
angular_speed_rads=(udds_speed_ms)/wheel_radius_m
wheel_rpm=angular_speed_rads*(60/2*math.pi)

# motor angular velocity from axle 9:1 ratio, applies to both motors
gear_ratio=9.036
motor_rpm=wheel_rpm*gear_ratio
motor_torque_Nm=wheel_torque_Nm/gear_ratio
power_motor_kW=motor_rpm*motor_torque_Nm*(2*math.pi/60)

plt.plot(udds_speed_mph,wheel_torque_Nm)
plt.show()

