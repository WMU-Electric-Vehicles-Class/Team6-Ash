from cmath import sqrt
import numpy as np
from matplotlib import pyplot as plt

#vehicle_weight_kg = 2232

#dds_speed_ms = udds_speed_mph/3.6

#Speed_motor_radsec = 1
#Motor_controlSignal = 100  #Percent
Torque_motor_Nm = 493
#Power_motor_in_W = 1
#Power_motor_out_W = Torque_motor_Nm*Speed_motor_radsec
#motor_efficiency = Power_motor_out_W/Power_motor_in_W

rpm= np.linspace(1,500)
torque=np.linspace(1,Torque_motor_Nm)

#motor loss constants
Kc= 0.2
Ki=0.008
Kw=0.00001
C=20            #Constant motor losses

[rpm,torque]=np.meshgrid(rpm,torque)

Output_power=(rpm*torque)

c_l=(torque**2)*Kc   #Copper losses
I_l=(rpm*Ki)      #Iron losses
W_l=(rpm**3)*Kw   #Winding and friction losses, assumption: 3-phase


Input_power= Output_power+c_l+I_l+W_l+C

Z=Output_power/Input_power

V=[.7,.8,.85,.9,.91,.92,.925,.93,.94,.95]  #efficiency lines
#plt.contourf(rpm,torque,Z,V)
#plt.colorbar()
#plt.xlabel('Motor Speed (rpm)')
#plt.ylabel('Motor Torque (Nm)')
#plt.show()

RM_Speed_rpm=np.array([200,100,500,12000,3000,1000,2000,6000,10000,4000,2000,10000,8700,2000,2000,4000,8000,1000,6000,4000,4000,6000,5000,3000])
RM_Torque_Nm=np.array([200,230,25,5,230,50,150,100,5,175,125,25,10,100,50,125,25,10,75,160,50,50,50,50])
#Speed_rpm=np.array([2000,1000,4000,4000,4000,2000,2000,8000,10000,6000,12000,10000,8700,1000,500,200,3000,100,6000,5000,3000,2000,6000,4000])
#Torque_Nm=np.array([50,50,50,125,175,125,150,25,25,100,5,5,10,10,25,200,230,230,50,50,50,100,75,160])

RM_efficiency=np.array([63,63,90,93,93,94,94,94,94,95,95,95,95,95,96,96,96,96,96,96,97,97,97,97])
#Output_power_kW=(Speed_rpm*Torque_Nm)
#efficiency=np.array([96,94,97,96,95,95,94,96,95,94,93,94,95,96,90,63,93,63,97,97,97,95,96,96])
#Input_power_kW=Output_power_kW/efficiency

#Speed_rpm,Torque_Nm=np.meshgrid(Speed_rpm,Torque_Nm)
#[efficiency]=np.meshgrid(efficiency)

plt.tricontourf(list(RM_Speed_rpm),list(RM_Torque_Nm),list(RM_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for IPM-synRM')
plt.show()

AC_Speed_rpm=np.array([250,1750,500,1500,1000,750,600,1750,750,1750,1500,1000,1000,1000,1750,1500,1250,1500,1750,2500,4750,400,100,100,4000,350,2750,2500,2500,500,250,250,750,1000,750,500])
AC_Torque_Nm=np.array([250,570,63,500,380,200,126,450,189.9,380,430,316,250,63,316,250,63,63,60,60,50,50,60,400,70,80,150,220,70,500,200,570,570,500,430,400])
AC_efficiency=np.array([70,82,85,85,85,86,87,87,89,89,89,89,90,91,91,91,92,92,93,94,92,93,20,50,92,92,92,92,94,72,72,50,72,78,80,87])

plt.tricontourf(list(AC_Speed_rpm),list(AC_Torque_Nm),list(AC_efficiency), levels=[60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,93,94,96,98])
plt.colorbar()
plt.xlabel('Motor Speed (rpm)')
plt.ylabel('Motor Torque (Nm)')
plt.title('Efficiency Map for AC Induction Motor')
plt.show()

#Using lecture equations
voltage=360

V1=voltage/sqrt(3)      #phase voltage
Rs= 1       #stator winding resistance
Xs= 3       #stator reactance
Xr= 8       #rotor reactance


p=6         #number of poles
f=60        #frequency in Hz
nslip_rpm=(120*f)/p         #AC motor synchronous speed
print(nslip_rpm)


# drive Cycle

udds = np.loadtxt('uddscol_.txt', dtype=int)               #udds data pull

udds_time_s = udds[:,0]
udds_speed_mph = udds[:,1]

wheel_radius_m=.2286
angular_speed_rad=(udds_speed_mph/2.237)/wheel_radius_m  #in rad/s

angular_speed_rpm=angular_speed_rad*(60/2*3.14)

angular_speed_rpm_motor=angular_speed_rpm/9     #from drivetrain to motor

print(angular_speed_rad)

print(np.interp(angular_speed_rpm,AC_Speed_rpm, AC_Torque_Nm))


