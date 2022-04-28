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
from scipy.signal import savgol_filter
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
udds_speed_mips=udds_speed_mph/3600
distance_m_udd = np.cumsum(udds_speed_ms)
distance_mi_udds = np.cumsum(udds_speed_mips)
#distance_mi_udds = distance_m_udd/1609

################ UDDS Drive Cycle - without First stop  ###################################
udds = np.loadtxt('uddscol_no_first_stop.txt', dtype=int)
udds_time_s_no_stop = udds[:, 0]
udds_speed_mph_no_stop = udds[:, 1]
udds_speed_ms_no_stop = udds_speed_mph_no_stop*.44704
udds_accel_ms2_no_stop = np.diff(udds_speed_ms_no_stop, prepend=0)
distance_m_udd_no_stop = np.cumsum(udds_speed_ms_no_stop)
Distance_mi_no_stop = distance_m_udd_no_stop/1609

################# HWY Drive Cycle ####################################
hwy = np.loadtxt('hwycol.txt', dtype=int)
hwy_time_s = hwy[:, 0]
hwy_speed_mph = hwy[:, 1]
hwy_speed_ms = hwy_speed_mph*.44704
hwy_accel_ms2 = np.diff(hwy_speed_ms, prepend=0)
distance_m_hwy = np.cumsum(hwy_speed_ms)
Distance_mi_hwy = distance_m_hwy/1609

########## Smooth HWY ################
smooth_hwy_ms=savgol_filter(hwy_speed_ms, 201, 3)
smooth_hwy_mph= smooth_hwy_ms/.44704
distance_m_hwy_smooth = np.cumsum(smooth_hwy_ms)
distance_mi_hwy_smooth= distance_m_hwy_smooth/1609
smooth_hwy_accel_ms2 = np.diff(smooth_hwy_ms, prepend=0)

plt.plot(hwy_time_s,hwy_speed_mph)
plt.plot(hwy_time_s,smooth_hwy_mph)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (mph)')
plt.legend(["Highway Drive Cycle Velocity", "Highway Drive Cycle Velocity Smoothed"])
plt.show()
print('distance', distance_m_hwy[len(distance_m_hwy)-1],distance_m_hwy_smooth[len(distance_m_hwy_smooth)-1])

################# NYCC Drive Cycle ####################################
nycc = np.loadtxt('nycccol_eco.txt', dtype=int)
nycc_time_s = nycc[:,0]
nycc_speed_mph = nycc[:,1]
nycc_speed_ms = nycc_speed_mph*.44704
nycc_accel_ms2 = np.diff(nycc_speed_ms, prepend=0)
distance_m_nycc = np.cumsum(nycc_speed_ms)
Distance_mi_nycc = distance_m_nycc/1609

########## Constant Velocity NYCC ################

nycc_Speed_limit = 20  # 15 mile/hr

nycc_eco = np.loadtxt('nycccol_eco.txt', dtype=int)
nycc_speed_mph_eco = nycc_eco[:,1]

nycc_speed_mph_l =[]
for i in nycc_speed_mph_eco:
    if i>nycc_Speed_limit:
        i = nycc_Speed_limit
    nycc_speed_mph_l.append(i)
nycc_speed_mph_cons=np.array(nycc_speed_mph_l)
nycc_speed_ms_cons= nycc_speed_mph_cons **.44704
distance_m_nycc_cons = np.cumsum(nycc_speed_ms_cons)
distance_mi_nycc_cons=distance_m_nycc_cons/1609
nycc_accel_ms2_cons = np.diff(nycc_speed_ms_cons, prepend=0)
plt.plot(nycc_time_s, nycc_speed_mph)
plt.plot(nycc_time_s, nycc_speed_mph_cons)
plt.xlabel('Time (s)')
plt.ylabel('Speed (mph)')
plt.legend(["NYCC Velocity", "NYCC Drive cycle with more constant velocity"])
plt.grid()
plt.show()

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
distance_m_udds_limited = np.cumsum(udds_speed_ms_limited)
distance_mi_udds_limited=distance_m_udds_limited/1609
udds_accel_ms2_limited = np.diff(udds_speed_ms_limited, prepend=0)
#print('udds distance',distance_m_udd[len(distance_m_udd)-1], distance_m_udds_limited[len(distance_m_udds_limited)-1])



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
        self.speed = udds_speed_ms
        self.acc = udds_accel_ms2
        self.radius= wheel_radius_m
        self.gear_ratio=gear_ratio
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
        #wheel_torque_Nm = F_prob*(self.radius) 

        Wheel_torque_list =[]
        for i in F_prob:
            Wheel_torque = i/2 * self.radius
            Wheel_torque_list.append(Wheel_torque)

        angular_speed_rads = (self.speed)/self.radius
        wheel_rpm = angular_speed_rads*(60/2*np.pi)
        
        motor_rpm = wheel_rpm*self.gear_ratio
        motor_torque_Nm = Wheel_torque/self.gear_ratio 

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
Final_SOC_Percent_UDDS = Battery.SOC(ss)

plt.plot(udds_time_s, Final_SOC_Percent_UDDS)
plt.xlabel('Time Cycle (s)')
plt.ylabel('SOC (%)')
plt.grid()
plt.show()

#####################################    SOC Vs With/without regenerative braking for UDDS    #####################################
s1z = Battery()
szb = Battery.D_soc(s1z)
Final_SOC_Percent_UDDS = Battery.SOC(szb)

sxz2 = Battery()
sxz2.reg = 0
szx2 = Battery.D_soc(sxz2)
Final_SOC_Percent_No_Reg_UDDS = Battery.SOC(szx2)
# print("SOC for without Gen UDDS:", Final_SOC_Percent_No_Reg_UDDS[-1])
# print("SOC for with Gen UDDS:", Final_SOC_Percent_UDDS[-1])


plt.plot(udds_time_s, Final_SOC_Percent_UDDS, Final_SOC_Percent_No_Reg_UDDS)
plt.legend(['With regenerative braking in UDDS Cycle',
           'Without regenerative braking in UDDS Cycle'])
plt.xlabel('Time Cycle(s)')
plt.ylabel('SOC(%)')
plt.grid()
plt.show()
##################################################### Normal speed vs limited speed   ###########################

plt.plot(distance_mi_udds, udds_speed_mph)
plt.plot(distance_mi_udds, udds_speed_mph_limited)
plt.xlabel('Distance (mi)')
plt.ylabel('Speed (mph)')
plt.legend(["UDDS Velocity", "UDDS Velocity Omitting Speeds Below 15 mph"])
plt.grid()
plt.show()

################################################   SOC vs without first stop   ###########################
d1=Battery()
d1.speed=udds_speed_ms_no_stop
d12=Battery.D_soc(d1)
Final_SOC_Percent_UDDS_no_first_stop=Battery.SOC(d12)

plt.plot(udds_time_s, udds_speed_ms,udds_speed_ms_no_stop)
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend(["UDDS Cycle", "UDDS without first stop"])
plt.grid()
plt.show()

plt.plot(udds_time_s_no_stop, Final_SOC_Percent_UDDS,Final_SOC_Percent_UDDS_no_first_stop)
plt.xlabel('Time (s)')
plt.ylabel('SOC (%)')
plt.legend(["SOC in UDDS Cycle", "SOC in UDDS Cycle without first stop"])
plt.grid()
plt.show()

#print("SOC for UDDS: ",Final_SOC_Percent_UDDS[-1],"\nSOC for UDDS no first stop: ",Final_SOC_Percent_UDDS_no_first_stop[-1] )

######################################################   SOC vs the speed limit   ###########################
sp = Battery()
sp.speed=udds_speed_ms_limited
sp.acc=udds_accel_ms2_limited
sp1 = Battery.D_soc(sp)
SOC_speed_limit = Battery.SOC(sp1)

plt.plot(distance_mi_udds, Final_SOC_Percent_UDDS)
plt.plot(distance_mi_udds, SOC_speed_limit)
plt.xlabel('Distance (mi)')
plt.ylabel('SOC (%)')
plt.legend(["UDDS Velocity", "UDDS Velocity Omitting Speeds Below 15 mph", ])
plt.grid()
plt.show()

#print(len(SOC_speed_limit))
#print(SOC_speed_limit[1369], Final_SOC_Percent_UDDS[1369])

######################################################   SOC vs Smooth Hwy   ###########################
c1=Battery()
c1.speed=hwy_speed_ms
c1.acc=hwy_accel_ms2
cc=Battery.D_soc(c1)
Final_SOC_Percent_Hwy=Battery.SOC(cc)

c2=Battery()
c2.speed=smooth_hwy_ms
c2.acc=smooth_hwy_accel_ms2
ccc=Battery.D_soc(c2)
Final_SOC_Percent_Hwy_smooth=Battery.SOC(ccc)

plt.plot(Distance_mi_hwy, Final_SOC_Percent_Hwy)
plt.plot(Distance_mi_hwy,Final_SOC_Percent_Hwy_smooth)
plt.xlabel('Distance (mi)')
plt.ylabel('SOC (%)')
plt.legend(["Highway Drive Cycle Velocity", "Highway Drive Cycle Velocity Smoothed"])
plt.grid()
plt.show()

#print(Final_SOC_Percent_Hwy[len(Final_SOC_Percent_Hwy)-1], Final_SOC_Percent_Hwy_smooth[len(Final_SOC_Percent_Hwy_smooth)-1])
#print(Distance_mi_hwy)

######################################################   SOC vs Constant Velocity NYCC   ###########################
n1=Battery()
n1.speed=nycc_speed_ms
n1.acc=nycc_accel_ms2
nn=Battery.D_soc(n1)
Final_SOC_Percent_nycc=Battery.SOC(nn)

n2=Battery()
n2.speed=nycc_speed_ms_cons
n2.acc=nycc_accel_ms2_cons
nnn=Battery.D_soc(n2)
Final_SOC_Percent_nycc_cons=Battery.SOC(nnn)

plt.plot(Distance_mi_nycc, Final_SOC_Percent_nycc)
plt.plot(Distance_mi_nycc,Final_SOC_Percent_nycc_cons)
plt.xlabel('Distance (mi)')
plt.ylabel('SOC (%)')
plt.legend(["NYCC Drive Cycle Velocity", "NYCC at more Constant Velocity"])
plt.grid()
plt.show()
print('SOC', Final_SOC_Percent_nycc[len(Final_SOC_Percent_nycc)-1],Final_SOC_Percent_nycc_cons[len(Final_SOC_Percent_nycc_cons)-1])

############################################################ SOC vs weight   #################################
s2 = Battery()
s2.v_mass = 1785   # 20 percent less
s2d = Battery.D_soc(s2)
Final_SOC_less_weight = Battery.SOC(s2d)

s3 = Battery()
s3.v_mass = 2678.4   # 20 percent more
s3d = Battery.D_soc(s3)
Final_SOC_more_weight = Battery.SOC(s3d)

plt.plot(udds_time_s, Final_SOC_Percent_UDDS)
plt.plot(udds_time_s, Final_SOC_more_weight)
plt.plot(udds_time_s, Final_SOC_less_weight)
plt.xlabel('Time Cycle (s)')
plt.ylabel('SOC (%)')
plt.legend(["Actual Weight", "20% Heavier","20% Lighter"])
plt.grid()
plt.show()

######################################## SOC vs Distance  ################################
plt.plot(distance_mi_udds, Final_SOC_Percent_UDDS)
plt.title('SOC (%) for UDDS')
plt.xlabel('Distance (mi)')
plt.ylabel('SOC (%)')
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
ax1.plot(x,Final_SOC_Percent_UDDS)
ax1.set_title("Tesla S & Tesla Model 3 SOC vs. Time", fontsize="large", fontweight="bold")
ax1.set_xlabel("Time [sec]", fontsize="large")
ax1.set_ylabel("SOC [%]", fontsize="large")
ax1.legend(["Tesla Model S60","Tesla model 3 (Long Range )"])
plt.show()
