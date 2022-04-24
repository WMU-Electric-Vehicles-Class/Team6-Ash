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
##################### Vehicle parameters  ##############################

empty_vehicle_weight_kg = 2232
g = 9.81
Crr = 0.015
Cd = 0.28
Air_Density = 1.2
Vehicle_Height_m = 1.443
Vehicle_Width_m = 1.849
wheel_radius_m = .2286
gear_ratio = 9.036
Internal_resistance= 0.320
Frontal_area = 2.34
Air_Density = 1.2
Battery_Cap_A_h = 205
Voltage_open_c_v = 400
regenerative_status = 1  # 0 = Off, 1 = On
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


########################### Efficiency map data points for AC Induction front motor #####################

AC_Speed_rpm = np.array([50, 300, 500, 500, 1000, 1000, 1500, 2000, 1000, 1500, 2000, 3000, 2000, 2500, 3000, 2000, 2500, 3000, 4000, 
                        4000, 3000, 3500, 4000, 5000, 8000, 10000, 12000, 4000, 5000, 6000, 8000, 10000, 12000, 10000, 12000, 
                        7000, 8000, 10000, 12000, 8000, 10000, 12000, 8000, 10000, 12000]) 
AC_Torque_Nm = np.array([250, 0, 50, 150, 300, 75, 250, 300, 50, 100, 150, 300, 75, 150, 250, 25, 75, 150, 250, 250, 50, 
                        125, 150, 250, 200, 140, 125, 50, 100, 200, 25, 25, 25, 125, 125, 125, 150, 100, 100,
                        50, 50, 50, 100, 75, 60])
AC_efficiency = np.array([20, 0, 82, 82, 82, 87, 86, 88, 88, 90, 90, 90, 91, 91, 91, 92, 92, 92, 92, 92, 93, 93, 93, 93, 93, 
                        93, 93, 94, 94, 94, 94, 94, 94, 94, 94, 95, 95, 95, 95, 95, 95, 95, 96, 96, 96])



####################### SOC
####################### SOC vs Distance
####################### SOC vs weight
####################### Normal speed vs limited speed
####################### Compare SOC with the speed limit
####################### Compare SOC with Fastsim Model

Speed_limit = 15  # 15 mile/hr

udds_speed_mph_l =[]
for i in udds_speed_mph:
    if i<Speed_limit:
        i = Speed_limit
    udds_speed_mph_l.append(i)
udds_speed_mph_limited=np.array(udds_speed_mph_l)
udds_speed_ms_limited= udds_speed_mph_limited **.44704

class Battery:
    def __init__(self):
        self.v_mass = empty_vehicle_weight_kg
        self.Coeff_drag = Cd
        self.Frontal_area = Frontal_area
        self.Coeff_rolling = Crr
        self.Air_Density = Air_Density
        self.R_int = Internal_resistance
        self.Battery_Cap_A_h = Battery_Cap_A_h
        self.Voltage_open_c_v = Voltage_open_c_v
        self.g = g
        self.radius = wheel_radius_m
        self.speed = udds_speed_ms
        self.acc = udds_accel_ms2
        self.reg = regenerative_status


    def D_soc(self):
        f1_list = []
        f2_list = []
        for v in self.speed:
            f1 = (0.5)*(self.Coeff_drag)*(self.Air_Density)*(self.Frontal_area) * \
                (v**2) + (self.Coeff_rolling)*(self.v_mass)*(self.g)
            f1_list.append(f1)
        for i in (self.acc):
            f2 = (self.v_mass * i)
            if f2 <= 0:
                f2 = f2*self.reg
            f2_list.append(f2)
        F_prob = [x + y for (x, y) in zip(f1_list, f2_list)]
        Power_vehicle = F_prob * self.speed
        Power_axle = Power_vehicle/2
        wheel_torque_Nm = Power_axle*self.radius

        angular_speed_rads = (self.speed)/self.radius
        wheel_rpm = angular_speed_rads*(60/2*np.pi)
        
        motor_rpm = wheel_rpm*gear_ratio
        motor_torque_Nm = wheel_torque_Nm/gear_ratio 

        RM_Eff_interpol = NearestNDInterpolator(
            (RM_Speed_rpm, RM_Torque_Nm), RM_efficiency)
        Motor_eff_rear_percent = RM_Eff_interpol(abs(motor_rpm), abs(motor_torque_Nm))
        Motor_eff_rear = Motor_eff_rear_percent/100

        AC_Eff_interpol = NearestNDInterpolator(
           (AC_Speed_rpm, AC_Torque_Nm), AC_efficiency)
        Motor_eff_front_percent = AC_Eff_interpol(abs(motor_rpm), abs(motor_torque_Nm))
        Motor_eff_front = Motor_eff_front_percent/100
        
        Pbatt_front_motor = Power_axle/Motor_eff_front
        Pbatt_rear_motor = Power_axle/Motor_eff_rear
        Power_battery = Pbatt_front_motor + Pbatt_rear_motor
        delta = (self.Voltage_open_c_v**2) - (4*Power_battery*self.R_int)
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

####################################        SOC with and without regenerative     #####################################
s1 = Battery()
ss = Battery.D_soc(s1)
Final_SOC_Percent = Battery.SOC(ss)

r1 = Battery()
r1.reg=0
r2 = Battery.D_soc(r1)
Final_SOC_Percent_without_reg = Battery.SOC(r2)

plt.plot(udds_time_s, Final_SOC_Percent, Final_SOC_Percent_without_reg)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.grid()
plt.show()

##################################################### Normal speed vs limited speed   ###########################

plt.plot(udds_time_s, udds_speed_mph, udds_speed_mph_limited)
plt.xlabel('Time Cycle(s)')
plt.ylabel('Speed ( mile/h)')
plt.legend(["UDDS velocity", "UDDS velocity for the speed above 15 mile/h"])
plt.grid()
plt.show()

######################################################   SOC vs the speed limit   ###########################
sp = Battery()
sp.speed=udds_speed_ms_limited
sp1 = Battery.D_soc(sp)
SOC_speed_limit = Battery.SOC(sp1)

plt.plot(udds_time_s, SOC_speed_limit, Final_SOC_Percent)
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.legend(["velocity limited to above 15 mile/h", "For all velocity in UDDS Cycle", ])
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
