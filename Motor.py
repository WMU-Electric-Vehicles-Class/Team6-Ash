
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.integrate import quad


################### Efficiency map data points for IPM-synRM rear motor ####################
RM_Speed_rpm=np.array([200,100,500,12000,3000,1000,2000,6000,10000,4000,2000,10000,8700,2000,2000,4000,8000,1000,6000,4000,4000,6000,5000,3000])
RM_Torque_Nm=np.array([200,230,25,5,230,50,150,100,5,175,125,25,10,100,50,125,25,10,75,160,50,50,50,50])
RM_efficiency=np.array([63,63,90,93,93,94,94,94,94,95,95,95,95,95,96,96,96,96,96,96,97,97,97,97])

plt.tricontourf(list(RM_Speed_rpm),list(RM_Torque_Nm),list(RM_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for IPM-synRM')
plt.show()

########################### Efficiency map data points for AC Induction front motor #####################
AC_Speed_rpm=np.array([250,1750,500,1500,1000,750,600,1750,750,1750,1500,1000,1000,1000,1750,1500,1250,1500,1750,2500,4750,400,100,100,4000,350,2750,2500,2500,500,250,250,750,1000,750,500])
AC_Torque_Nm=np.array([250,570,63,500,380,200,126,450,189.9,380,430,316,250,63,316,250,63,63,60,60,50,50,60,400,70,80,150,220,70,500,200,570,570,500,430,400])
AC_efficiency=np.array([70,82,85,85,85,86,87,87,89,89,89,89,90,91,91,91,92,92,93,94,92,93,20,50,92,92,92,92,94,72,72,50,72,78,80,87])

plt.tricontourf(list(AC_Speed_rpm),list(AC_Torque_Nm),list(AC_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for AC Induction Motor')
plt.show()

################ UDDS Drive Cycle  ###################################
udds = np.loadtxt('uddscol_.txt', dtype=int)               
udds_time_s = udds[:,0]
udds_speed_mph = udds[:,1]
udds_speed_ms=udds_speed_mph*.44704
udds_accel_ms2=np.diff(udds_speed_ms,prepend=0)

##################### Vehicle parameters  ##############################
empty_vehicle_weight_kg=2232
g=9.81

###################  Force due to mass of the vehicel (Fm) in Newton ########################
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

################## torque per wheel in Nm ###########################
wheel_torque_Nm=Fw*wheel_radius_m

# angular velocity of wheel
angular_speed_rads=(udds_speed_ms)/wheel_radius_m
wheel_rpm=angular_speed_rads*(60/2*math.pi)

############### motor angular velocity from axle 9:1 ratio, applies to both motors  #######################
gear_ratio=9.036
motor_rpm=wheel_rpm*gear_ratio
motor_torque_Nm=wheel_torque_Nm/gear_ratio
power_motor_kW=motor_rpm*motor_torque_Nm*(2*math.pi/60)
power_motor_W=power_motor_kW/1000


###################### Battery Parameters #################################

battery_Voltage_max = 400
battery_Voltage = 360
SOC=1.0
SOC_min=0.1
Internal_resistance = 0.320
max_Energy_Capacity_kw =82
min_Energy_Capacity_kw =7
SOC_min = min_Energy_Capacity_kw/max_Energy_Capacity_kw
Ctb = 82000
CD = 0.28
RO = 1.19
A = 2.34
CRR = 0.015
Rbi = 0.320
M = 2232
g = 9.81
SOC_in =1
Pb_list=[]
I0_list =[]
SOC_list = []
t0 = 0
t1 = 1369



# #############   Battery Power   #########

for v in (udds_speed_ms):
    Pb = ((0.5)*(CD)*(RO)*(A)*(v**2)+(CRR)*(M)*(g))*v
    Pb_list.append(Pb)

    
# #############   Battery Current   #########
for w in (Pb_list):
    I0=w/360
    I0_list.append(I0)

for I in (I0_list):
    SOC = (SOC - ((I/Ctb)*(0.5)*((t1**2)-(t0**2))))
    SOC_list.append(SOC)



# #########################   SOC    ######################################

dSOC =1
SOC_list = []
I1_list =[]
for power in (Pb_list):
    sq = ((battery_Voltage_max)-(abs(battery_Voltage_max**2)-4*Rbi*power)**0.5)
    # if sq < 0 :
    #     sq*(-1)
    #     continue
    I1 = sq/(2*Rbi)
    I1_list.append(sq)

    for I in (I1_list):
        dSOC = -(I/3600)/Ctb     
        SOC = SOC + (I/Ctb)*dSOC
        # if SOC < SOC_min:
        #     SOC=0
        # else:
        SOC_list.append(SOC)
   
##############################  Plot Current, Power, SOC  VS time  #####################

plt.plot(udds_time_s,I0_list)
plt.xlabel('time(s)')
plt.ylabel('Current(A)')
plt.grid()
plt.show()

# Plot Power vs time
plt.plot(udds_time_s,Pb_list)
plt.xlabel('time (s)')
plt.ylabel('Power (w)')
plt.grid()
plt.show()

plt.plot(udds_time_s,udds_speed_ms)
plt.xlabel('time(s)')
plt.ylabel('Speed (m/s)')
plt.grid()
plt.show()


# plt.plot(udds_time_s,SOC_list)
# plt.xlabel('time')
# plt.ylabel('SOC(%)')
# plt.grid()
# plt.show()

# ##########################   Argonne  Benze ###################################

Argonne = np.loadtxt('61512023_Test_Data_argonee_Benz.txt', dtype=int)               
Argonne_time_s = Argonne[:,0]
Argonne_speed_mph = Argonne[:,4]
Argonne_Current =Argonne[:,7]
Argonne_Voltage =Argonne[:,8]
Argonne_SOC =Argonne[:,9]
Argonne_Motor_speed=Argonne[:,10]
Argonne_speed_ms=Argonne_speed_mph*.44704
Argonne_accel_ms2=np.diff(Argonne_speed_ms,prepend=0)
Argonne_power_w= Argonne_Current*Argonne_Voltage

#Plot Our Current VS argonne vs time
# plt.plot(Argonne_time_s,SOC2_list)
# plt.plot(Argonne_time_s,Argonne_SOC)
# plt.xlabel('time')
# plt.ylabel('SOC')
# plt.grid()
# plt.show()

plt.plot(Argonne_time_s,Argonne_Current)
plt.plot(udds_time_s,I0_list)
plt.xlabel('time(s)')
plt.ylabel('Current(A)')
plt.legend(('2015 Mercedes-Benz B-Class', '2019 Tesla Model 3 (Long Range)'), loc='upper right', shadow=True)
plt.grid()
plt.show()



plt.plot(Argonne_time_s,Argonne_power_w)
plt.plot(udds_time_s,Pb_list)
plt.xlabel('time(s)')
plt.ylabel('Output Power(W)')
plt.legend(('2015 Mercedes-Benz B-Class', '2019 Tesla Model 3 (Long Range)'), loc='upper right', shadow=True)
plt.grid()
plt.show()


####################################### Regenerative braking ################################################


Speed_list= []
Eff_list = []
m = 2232   # Mass
#v1 = 120
v2 = 0
a = -10   #decelerate
Voltage = 400
Current = 187.5
eff = 0.8*0.8*0.7
Pb = (Voltage*Current)   #    MAX_Absorbtion_Energy

for i in range (20,151):
    v1=i
    Speed_list.append(v1)
    KE = ((0.5*m*((v1/3.6)**2))- (0.5*m*v2))/1000
    t0 = (v2-(v1/3.6))/a
    t_star = t0 - ((Pb)/(eff*m*(a**2)))

    def integrand(t, a):
        return a*t
    I0 = quad(integrand, t0, t_star, args=(a))
    I1 = sum(list(I0))
    I = (Pb*t_star)
    Eb = (I +(eff*m*a*(-1)*I1))/1000
    Eff= (Eb/KE)*100
    Eff_list.append(Eff)

plt.plot(Speed_list,Eff_list)
plt.xlabel('Velocity (km/h)')
plt.ylabel('Recuperated Energy Efficiency (%)')
plt.grid()
plt.show()

############################ Efficiency of recuperated energy vs decelerates ################################################

decelerate_list= []
Eff_list = []
m = 2232   # Mass
v1 = 100
v2 = 0
Voltage = 360
Current = 187
eff = 0.8*0.8*0.7
Pb = (Voltage*Current)   #    MAX_Absorbtion_Energy

for i in range (-30,-1):
    a=i
    decelerate_list.append(i)
    KE = ((0.5*m*(v1/3.6)**2)- (0.5*m*v2))/1000
    t = (v2-v1/3.6)/a
    t_star = t - ((Pb)/(eff*m*(a**2)))
    # The energy stored in the battery is:
    Eb =  ((Pb*t_star)+(eff*m*a*(a/2)*(t**2 - t_star**2 )))/1000
#   Energy_Efficiency 
    Eff= (Eb/KE)*100
    Eff_list.append(Eff)
    
plt.plot(decelerate_list,Eff_list)
plt.xlabel('decelerate (m/s)')
plt.ylabel('Recuperated Energy Efficiency (%)')
plt.grid()
plt.show()
