#Vehicle Model Name
input("Vehicle Model Name = ")

#Vehicle Battery Energy in 'kWh'
Eb = float(input("Vehicle Battery Enery in kWh = "))

#SOC (State of Charge) in Percentage
Soc = float(input("SOC in Percentage = "))

#HVAC Auxil1ary Power Load in 'W'
Pl = float(input("Auxiliary Power Load W = "))

t = ((Eb*10*Soc)/Pl)
print("Time in hrs = ",t)

