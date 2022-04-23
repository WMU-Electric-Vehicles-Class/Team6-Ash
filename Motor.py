import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import LinearNDInterpolator
from scipy.integrate import quad
from scipy.interpolate import NearestNDInterpolator
from turtle import color
from fastsim import simdrive, vehicle, cycle
import sys
import os
from pathlib import Path
import time
import pandas as pd
import importlib

################ UDDS Drive Cycle  ###################################
udds = np.loadtxt('uddscol_.txt', dtype=int)
udds_time_s = udds[:, 0]
udds_speed_mph = udds[:, 1]
udds_speed_ms = udds_speed_mph*.44704
udds_accel_ms2 = np.diff(udds_speed_ms, prepend=0)
distance_m_udd = np.cumsum(udds_speed_ms)
Distance_mi = distance_m_udd/1609

################### Efficiency map data points for IPM-synRM rear motor ####################
RM_Speed_rpm = np.array([200, 100, 500, 12000, 3000, 1000, 2000, 6000, 10000, 4000,
                        2000, 10000, 8700, 2000, 2000, 4000, 8000, 1000, 6000, 4000, 4000, 6000, 5000, 3000])
RM_Torque_Nm = np.array([200, 230, 25, 5, 230, 50, 150, 100, 5,
                        175, 125, 25, 10, 100, 50, 125, 25, 10, 75, 160, 50, 50, 50, 50])
RM_efficiency = np.array([63, 63, 90, 93, 93, 94, 94, 94, 94,
                         95, 95, 95, 95, 95, 96, 96, 96, 96, 96, 96, 97, 97, 97, 97])

plt.tricontourf(list(RM_Speed_rpm), list(RM_Torque_Nm), list(RM_efficiency), levels=[
                60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 93, 94, 96, 98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for IPM-synRM')
plt.show()

########################### Efficiency map data points for AC Induction front motor #####################

AC_Speed_rpm = np.array([50, 300, 500, 500, 1000, 1000, 1500, 2000, 1000, 1500, 2000, 3000, 2000, 2500, 3000, 2000, 2500, 3000, 4000, 
                        4000, 3000, 3500, 4000, 5000, 8000, 10000, 12000, 4000, 5000, 6000, 8000, 10000, 12000, 10000, 12000, 
                        7000, 8000, 10000, 12000, 8000, 10000, 12000, 8000, 10000, 12000]) 
AC_Torque_Nm = np.array([250, 0, 50, 150, 300, 75, 250, 300, 50, 100, 150, 300, 75, 150, 250, 25, 75, 150, 250, 250, 50, 
                        125, 150, 250, 200, 140, 125, 50, 100, 200, 25, 25, 25, 125, 125, 125, 150, 100, 100,
                        50, 50, 50, 100, 75, 60])
AC_efficiency = np.array([20, 0, 82, 82, 82, 87, 86, 88, 88, 90, 90, 90, 91, 91, 91, 92, 92, 92, 92, 92, 93, 93, 93, 93, 93, 
                        93, 93, 94, 94, 94, 94, 94, 94, 94, 94, 95, 95, 95, 95, 95, 95, 95, 96, 96, 96])

plt.tricontourf(list(AC_Speed_rpm),list(AC_Torque_Nm),list(AC_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for AC Induction Motor')
plt.show()


##################### Vehicle parameters  ##############################
empty_vehicle_weight_kg = 2232
g = 9.81

###################  Force due to mass of the vehicle (Fm) in Newton ########################
Fm = empty_vehicle_weight_kg*udds_accel_ms2

# Force due to Rolling Resistance (Frr) in Newton
# Crr = Rolling Resistance Coefficient
Crr = 0.015  # Assumed from example probelm
Frr = Crr*empty_vehicle_weight_kg*g

# Aerodynamic force (Fd) in Newton
# Cd=Drag Coefficient
Cd = 0.28
Air_Density = 1.2
Vehicle_Height_m = 1.443
Vehicle_Width_m = 1.849
Vehicle_Front_Area_m2 = Vehicle_Height_m*Vehicle_Width_m
Fd = 0.5*Cd*Air_Density*Vehicle_Front_Area_m2*udds_speed_ms**2

# Total Force (Fprop) in Newton
Fprop = Fd+Frr+Fm

# Force in each axle (Fw) in Newton
Faxle = Fprop/2

# Wheel parameters with 18" diameter
wheel_radius_m = .2286

################## torque per wheel in Nm ###########################
wheel_torque_Nm = Faxle*wheel_radius_m

# angular velocity of wheel
angular_speed_rads = (udds_speed_ms)/wheel_radius_m
wheel_rpm = angular_speed_rads*(60/2*math.pi)

############### motor angular velocity from axle 9:1 ratio, applies to both motors  #######################
gear_ratio = 9.036
motor_rpm = wheel_rpm*gear_ratio
motor_torque_Nm = wheel_torque_Nm/gear_ratio

# plt.plot(motor_rpm, motor_torque_Nm)
# plt.show()

############## Motor Power_Output Power #######################
Pmotor_kW = motor_rpm*motor_torque_Nm*(2*math.pi/60)

############## Interpolating Motor Efficiency_IPM-synRM rear motor #######################
RM_Eff_interpol = NearestNDInterpolator(
    (RM_Speed_rpm, RM_Torque_Nm), RM_efficiency)
Motor_eff_rear_percent = RM_Eff_interpol(abs(motor_rpm), abs(motor_torque_Nm))
Motor_eff_rear = Motor_eff_rear_percent/100
# print(Motor_eff_rear)

############## Battery Power_Input Power #######################
# Pbatt_kW = Pmotor_kW/Motor_eff_rear
# Pbatt_W = Pbatt_kW*1000

# plt.plot(motor_rpm, Pmotor_kW)
# plt.plot(motor_rpm, Pbatt_kW)
# plt.show()



####################### SOC
####################### SOC vs Distance
####################### SOC vs weight
####################### Normal speed vs limited speed
####################### Compare SOC with the speed limit
####################### Compare SOC with Fastsim Model

Speed_limit = 8.94  # 8.94 m/s = 20 mile/hour

udds_speed_ms_l =[]
for i in udds_speed_ms:
    if i>Speed_limit:
        i = Speed_limit
    udds_speed_ms_l.append(i)
udds_speed_ms_under_limited=np.array(udds_speed_ms_l)


class Battery:
    def __init__(self):
        self.v_mass = 2232
        self.Coeff_drag = 0.259
        self.Frontal_area = 2.34
        self.Coeff_rolling = 0.008
        self.Air_Density = 1.2
        self.R_int = 0.32
        self.Battery_Cap_A_h = 205
        self.Voltage_open_c_v = 400
        self.g = 9.81
        self.speed = udds_speed_ms
        self.acc = udds_accel_ms2

    def D_soc(self):
        f1_list = []
        f2_list = []
        for v in self.speed:
            f1 = (0.5)*(self.Coeff_drag)*(self.Air_Density)*(self.Frontal_area) * \
                (v**2) + (self.Coeff_rolling)*(self.v_mass)*(self.g)
            f1_list.append(f1)
        for i in (self.acc):
            f2 = (self.v_mass * i)
            # if f2 <= 0:
            #     f2 = 0
            f2_list.append(f2)
        F_prob = [x + y for (x, y) in zip(f1_list, f2_list)]
        battery_power = F_prob * self.speed
        Real_power_battery = battery_power * Motor_eff_rear
        delta = (self.Voltage_open_c_v**2) - (4*Real_power_battery*self.R_int)
        b_current = (self.Voltage_open_c_v -
                     (np.sqrt(abs(delta))))/(2*self.R_int)
        dsoc = b_current*(1/3600)*(-1/self.Battery_Cap_A_h)
        return dsoc

    def SOC(dsoc):
        soclist = []
        soc = 0.9
        for i in dsoc:
            soc = (soc + i)
            if soc <= 0.1:
                soc = 0
            if soc > 1:
                soc = 1
            soclist.append(soc*100)
        return (soclist)




###################################################         SOC      #####################################
s1 = Battery()
ss = Battery.D_soc(s1)
Final_SOC_Percent = Battery.SOC(ss)

plt.plot(udds_time_s, Final_SOC_Percent)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.grid()
plt.show()

##################################################### Normal speed vs limited speed   ###########################

plt.plot(udds_time_s, udds_speed_ms, udds_speed_ms_under_limited)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.legend(["UDDS velocity", "UDDS velocity that limited to under 10 m/s"])
plt.grid()
plt.show()

######################################################   SOC vs the speed limit   ###########################
sp = Battery()
sp.speed=udds_speed_ms_under_limited
sp1 = Battery.D_soc(sp)
SOC_speed_limit = Battery.SOC(sp1)

plt.plot(udds_time_s, SOC_speed_limit, Final_SOC_Percent)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.legend(["velocity limited to under 10 m/s", "For all velocity", ])
plt.grid()
plt.show()

############################################################ SOC vs weight   #################################
s2 = Battery()
s2.v_mass = 1785   # 20 percent less
s2d = Battery.D_soc(s2)
Final_SOC_less_weight = Battery.SOC(s2d)

s3 = Battery()
s3.v_mass = 2678.4   # 20 percent more
s3d = Battery.D_soc(s3)
Final_SOC_more_weight = Battery.SOC(s3d)

plt.plot(udds_time_s, Final_SOC_Percent, Final_SOC_less_weight)
plt.plot(udds_time_s, Final_SOC_more_weight)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.legend(["Actual weight", "20 % lighter", "20 % Heavier"])
plt.grid()
plt.show()

######################################## SOC vs Distance  ################################
plt.plot(Distance_mi, Final_SOC_Percent)
plt.title('SOC(%) for UDDS')
plt.xlabel('Distance in miles')
plt.ylabel('SOC(%)')
plt.grid()
plt.show()

###################################   Fastsim MOdel   ######################

veh = vehicle.Vehicle(22)
veh.Scenario_name

cyc = cycle.Cycle("udds")
sim = simdrive.SimDriveClassic(cyc, veh)
sim.sim_drive()
print("soc:", sim.soc)
x = cyc.cycSecs
y = (sim.soc)*100

fig = plt.figure(figsize=(6, 4))
ax1 = fig.add_subplot()
ax1.plot(x, y)
ax1.plot(x,Final_SOC_Percent)
ax1.set_title("Tesla S & Tesla Model 3 SOC vs. Time", fontsize="large", fontweight="bold")
ax1.set_xlabel("Time [sec]", fontsize="large")
ax1.set_ylabel("SOC [%]", fontsize="large")
ax1.legend(["Tesla Model S60","Tesla model 3 (Long Range )"])
plt.show()
