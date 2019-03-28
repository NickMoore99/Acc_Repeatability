#--------------------------------------------------------------------------
# Created by Brandon Coats and Nick Moore
# Material Handling Services
# Linear Latch Cart Safety Mechanism
# -------------------------------------------------------------------------

import smbus
import csv
import math
import numpy as np
from scipy import stats
import sys
import time
from time import sleep
from time import gmtime, strftime
from REST.robot_functions import Mir
from REST.fleet_functions import Fleet
import matplotlib.pyplot as plt

	
def i2c_read_voltage(addr,ch):
	# Send configuration command
	try:
		bus.write_byte(address, ch[0])
	except:
		print("Failed to connect to MCP3426")
	# Sleep momentarily to allow settle
	sleep(0.25)
	# Read data back from 0x00(0), 2 bytes
	data = bus.read_i2c_block_data(addr, 0x00, 2)
	# Convert the data to 12-bits
	bit_adc = (data[0] & 0x0F) * 256 + data[1]
	if bit_adc > 2047 :
		bit_adc -= 4095
	# Convert and calibrate bit data to voltage
	volt_adc = bit_adc/2047*10*(ch[1]) #2047 represents 10v signal with correction factor
	# Output data to screen
	#print("ADC for Voltage ("+ch[2]+") : "+str(volt_adc))
	return (volt_adc)
  
def Write(IP,csvfilename):

	#initializations of data lists

	Sensor_Xp = [] 
	Sensor_YpXm = []
	Sensor_YpXp = []
	Sensor_Xm = []
	adj_Sensor_Xp = []
	adj_Sensor_YpXm = []
	adj_Sensor_YpXp = []
	adj_Sensor_Xm = []
	Theta = []
	
	timetoken = strftime("%Y%m%d%H%M", gmtime())
	
	f_out = open(csvfilename, 'a', newline='') #Open file for export
	f_in =  open(csvfilename, 'r')                    #Open file for import
	with f_out as csvfile:
		datawriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		with f_in:
			reader = csv.reader(f_in)
			num_rows = 0 # Row counter initialization
			for row in reader:
				# Read existing data and append to list
				if num_rows > 0:
					# Raw Sensor Data
					print(num_rows)
					print(len(Sensor_Xp))
					Sensor_Xp.append(float(row[1]))
					Sensor_YpXm.append(float(row[6]))
					Sensor_YpXp.append(float(row[11]))
					Sensor_Xm.append(float(row[15]))
					# Adjusted Sensor Data
					adj_Sensor_Xp.append(float(row[2]))
					adj_Sensor_YpXm.append(float(row[7]))
					adj_Sensor_YpXp.append(float(row[12]))
					adj_Sensor_Xm.append(float(row[16]))
					# Graph Data
					Theta.append(float(row[19]))
				num_rows += 1
			if num_rows == 0: # If no file already exists, create one with a header row
				print("Started new file!")
				datawriter.writerow(["DataIndex","Sensor_Xplus","Adjusted Val","Average","Corrected Val","Average","Sensor_YpXminus","Adjusted Val","Average","Corrected Val","Average","Sensor_YpXplus","Adjusted Val","Corrected Val","Average","Sensor_Xminus","Adjusted Val","Corrected Val","Average","Theta","Average","Corrected Theta","Average","Std","USL","LSL","Cpk","X axis","Average","Std","USL","LSL","Cpk","Y axis","Average","Std","USL","LSL","Cpk","Timetoken"])
		f_out.close()
		f_in.close()

	if num_rows > 0:
		index = num_rows - 1
	else:
		index = num_rows
	
	# Read voltage from ch1 sensor
	sens_volt_adc = i2c_read_voltage(address, ch1)
	Xp_val = sens_volt_adc
	# Read voltage from ch2 sensor
	sens_volt_adc = i2c_read_voltage(address, ch2)
	YpXm_val = sens_volt_adc
	# Read voltage from ch3 sensor
	sens_volt_adc = i2c_read_voltage(address, ch3)
	YpXp_val = sens_volt_adc
	# Read voltage from ch4 sensor
	sens_volt_adc = i2c_read_voltage(address, ch4)
	Xm_val = sens_volt_adc

        # Sensor raw readings
	Sensor_Xp.append((Xp_val*sensor_read_slope)+sensor_read_intercept)
	Sensor_YpXm.append(((YpXm_val*sensor_read_slope)+sensor_read_intercept)*math.cos(math.pi/4))
	Sensor_YpXp.append(((YpXp_val*sensor_read_slope)+sensor_read_intercept)*math.cos(math.pi/4))
	Sensor_Xm.append((Xm_val*sensor_read_slope)+sensor_read_intercept)
	
	dist = 43.875 # distance between side sensors, inches
	xp_bar = 22.75 # distance from center to laser Xp direction
	xm_bar = 21.25 # distance from center to laser Xm direction
	yp_bar = 12.375 #distance from center to laser Yp direction
	ym_bar = 14.5 # distance from center to laser Ym direction

	# Rotational Adjustment
	Theta.append((math.degrees(math.atan((Sensor_YpXm[index] - Sensor_YpXp[index])/dist))))
	Theta_C = math.radians(Theta[index])
	adj1 = Sensor_YpXp[index]+((math.tan(Theta_C))*(xp_bar+0.43*(math.sin(Theta_C/2))))
	adj2 = Sensor_YpXm[index]-((math.tan(Theta_C))*(xm_bar+0.43*(math.sin(Theta_C/2))))
	adj3 = Sensor_Xp[index]-((math.tan(Theta_C))*(yp_bar-0.65*(math.sin(Theta_C/2))))
	adj4 = Sensor_Xm[index]-((math.tan(Theta_C))*(ym_bar-0.65*(math.sin(Theta_C/2))))
	adj_Sensor_YpXp.append(float(adj1))
	adj_Sensor_YpXm.append(float(adj2))
	adj_Sensor_Xp.append(float(adj3))
	adj_Sensor_Xm.append(float(adj4))
	print("Theta: ",Theta[index])
	print("1: ",Sensor_Xp[index])
	print("2: ",Sensor_YpXm[index])
	print("3: ",Sensor_YpXp[index])
	print("4: ",Sensor_Xm[index])
	print("1a: ",adj3)
	print("2a:",adj2)
	print("3a: ",adj1)
	print("4a: ",adj4)
	
	if index > 10:

		C_Xp_Val = []
		C_YpXm_Val = []
		C_YpXp_Val = []
		C_Xm_Val = []
		C_Theta = []
		X_axis = []
		Y_axis = []
		f_in =  open(csvfilename, 'r')
		with f_in:
			reader = csv.reader(f_in)
			num_rows_2 = 0
			for row in reader:
				if num_rows_2 > 0:
					C_Xp_Val.append(float(row[4]))
					C_YpXm_Val.append(float(row[9]))
					C_YpXp_Val.append(float(row[13]))
					C_Xm_Val.append(float(row[17]))
					C_Theta.append(float(row[21]))
					X_axis.append(float(row[27]))
					Y_axis.append(float(row[33]))
				num_rows_2 += 1

		Adj_Xp_Avg = np.mean(adj_Sensor_Xp)
		C_Xp_Val.append(adj_Sensor_Xp[index] - Adj_Xp_Avg)
		C_Xp_Avg = np.mean(C_Xp_Val)
		
		Adj_YpXm_Avg = np.mean(adj_Sensor_YpXm)
		C_YpXm_Val.append(adj_Sensor_YpXm[index] - Adj_YpXm_Avg)
		C_YpXm_Avg = np.mean(C_YpXm_Val)

		Adj_YpXp_Avg = np.mean(adj_Sensor_YpXp)
		C_YpXp_Val.append(adj_Sensor_YpXp[index] - Adj_YpXp_Avg)
		C_YpXp_Avg = np.mean(C_YpXp_Val)			

		Adj_Xm_Avg = np.mean(adj_Sensor_Xm)
		C_Xm_Val.append(adj_Sensor_Xm[index] - Adj_Xm_Avg)
		C_Xm_Avg = np.mean(C_Xm_Val)
		
		counter = 0
		
		while abs(C_Xm_Val[index]) > 0.5 or abs(C_Xp_Val[index]) > 0.5 or abs(C_YpXm_Val[index]) > 0.2 or abs(C_YpXm_Val[index]) > 0.2:
			print("Retaking Value...")
			Sensor_Xp.remove[index]
			Sensor_YpXm.remove[index]
			Sensor_YpXp.remove[index]
			Sensor_Xm.remove[index]
			adj_Sensor_YpXp.remove[index]
			adj_Sensor_YpXm.remove[index]
			adj_Sensor_Xp.remove[index]
			adj_Sensor_Xm.remove[index]
			C_Xp_Val.remove[index]
			C_YpXm_Val.remove[index]
			C_YpXp_Val.remove[index]
			C_Xm_Val.remove[index]
			
			# Read voltage from ch1 sensor
			sens_volt_adc = i2c_read_voltage(address, ch1)
			Xp_val = sens_volt_adc
			# Read voltage from ch2 sensor
			sens_volt_adc = i2c_read_voltage(address, ch2)
			YpXm_val = sens_volt_adc
			# Read voltage from ch3 sensor
			sens_volt_adc = i2c_read_voltage(address, ch3)
			YpXp_val = sens_volt_adc
			# Read voltage from ch4 sensor
			sens_volt_adc = i2c_read_voltage(address, ch4)
			Xm_val = sens_volt_adc
			
			 # Sensor raw readings
			Sensor_Xp.append((Xp_val*sensor_read_slope)+sensor_read_intercept)
			Sensor_YpXm.append(((YpXm_val*sensor_read_slope)+sensor_read_intercept)*math.cos(math.pi/4))
			Sensor_YpXp.append(((YpXp_val*sensor_read_slope)+sensor_read_intercept)*math.cos(math.pi/4))
			Sensor_Xm.append((Xm_val*sensor_read_slope)+sensor_read_intercept)
	
			dist = 43.875 # distance between side sensors, inches
			xp_bar = 22.75 # distance from center to laser Xp direction
			xm_bar = 21.25 # distance from center to laser Xm direction
			yp_bar = 12.375 #distance from center to laser Yp direction
			ym_bar = 14.5 # distance from center to laser Ym direction

			# Rotational Adjustment
			Theta.append((math.degrees(math.atan((Sensor_YpXm[index] - Sensor_YpXp[index])/dist))))
			Theta_C = math.radians(Theta[index])
			adj1 = Sensor_YpXp[index]+((math.tan(Theta_C))*(xp_bar+0.43*(math.sin(Theta_C/2))))
			adj2 = Sensor_YpXm[index]-((math.tan(Theta_C))*(xm_bar+0.43*(math.sin(Theta_C/2))))
			adj3 = Sensor_Xp[index]-((math.tan(Theta_C))*(yp_bar-0.65*(math.sin(Theta_C/2))))
			adj4 = Sensor_Xm[index]-((math.tan(Theta_C))*(ym_bar-0.65*(math.sin(Theta_C/2))))
			adj_Sensor_YpXp.append(float(adj1))
			adj_Sensor_YpXm.append(float(adj2))
			adj_Sensor_Xp.append(float(adj3))
			adj_Sensor_Xm.append(float(adj4))
			
			Adj_Xp_Avg = np.mean(adj_Sensor_Xp)
			C_Xp_Val.append(adj_Sensor_Xp[index] - Adj_Xp_Avg)
			C_Xp_Avg = np.mean(C_Xp_Val)
		
			Adj_YpXm_Avg = np.mean(adj_Sensor_YpXm)
			C_YpXm_Val.append(adj_Sensor_YpXm[index] - Adj_YpXm_Avg)
			C_YpXm_Avg = np.mean(C_YpXm_Val)

			Adj_YpXp_Avg = np.mean(adj_Sensor_YpXp)
			C_YpXp_Val.append(adj_Sensor_YpXp[index] - Adj_YpXp_Avg)
			C_YpXp_Avg = np.mean(C_YpXp_Val)			

			Adj_Xm_Avg = np.mean(adj_Sensor_Xm)
			C_Xm_Val.append(adj_Sensor_Xm[index] - Adj_Xm_Avg)
			C_Xm_Avg = np.mean(C_Xm_Val)
			
			if counter > 2:
				break
			
			counter += 1
		
		
		X_axis.append((C_Xp_Val[index]-C_Xm_Val[index])/2)
		Y_axis.append((C_YpXp_Val[index]+C_YpXm_Val[index])/2)
		
		Theta_Avg = np.mean(Theta)
		C_Theta.append(Theta[index] - Theta_Avg)
		C_Theta_Avg = np.mean(C_Theta)
		Theta_Std = np.std(C_Theta)
		if C_Theta_Avg >= 0:
			USL_Theta = C_Theta_Avg + Theta_Std*6			
			LSL_Theta = -USL_Theta
		else:
			LSL_Theta = C_Theta_Avg - Theta_Std*6
			USL_Theta = -LSL_Theta
		Theta_Cpk_0 = (C_Theta_Avg-LSL_Theta)/(3*Theta_Std)
		Theta_Cpk_1 = (USL_Theta-C_Theta_Avg)/(3*Theta_Std)
		if Theta_Cpk_0 < Theta_Cpk_1:
			Theta_Cpk = Theta_Cpk_0
		else:
			Theta_Cpk = Theta_Cpk_1

		X_axis_Avg = np.mean(X_axis)
		X_axis_Std = np.std(X_axis)
		if X_axis_Avg >= 0:
			USL_X_axis = X_axis_Avg + X_axis_Std*6			
			LSL_X_axis = -USL_X_axis
		else:
			LSL_X_axis = X_axis_Avg - X_axis_Std*6
			USL_X_axis = -LSL_X_axis
		X_axis_Cpk_0 = (X_axis_Avg-LSL_X_axis)/(3*X_axis_Std)
		X_axis_Cpk_1 = (USL_X_axis-X_axis_Avg)/(3*X_axis_Std)
		if X_axis_Cpk_0 < X_axis_Cpk_1:
			X_axis_Cpk = X_axis_Cpk_0
		else:
			X_axis_Cpk = X_axis_Cpk_1
		
		Y_axis_Avg = np.mean(Y_axis)
		Y_axis_Std = np.std(Y_axis)
		if Y_axis_Avg >= 0:
			USL_Y_axis = Y_axis_Avg + Y_axis_Std*6			
			LSL_Y_axis = -USL_Y_axis
		else:
			LSL_Y_axis = Y_axis_Avg - Y_axis_Std*6
			USL_Y_axis = -LSL_Y_axis
		Y_axis_Cpk_0 = (Y_axis_Avg-LSL_Y_axis)/(3*Y_axis_Std)
		Y_axis_Cpk_1 = (USL_Y_axis-Y_axis_Avg)/(3*Y_axis_Std)
		if Y_axis_Cpk_0 < Y_axis_Cpk_1:
			Y_axis_Cpk = Y_axis_Cpk_0
		else:
			Y_axis_Cpk = Y_axis_Cpk_1
		
		
		f_out = open(csvfilename, 'a', newline='')
		with f_out as csvfile:
			datawriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			
			datawriter.writerow([index,Sensor_Xp[index],adj_Sensor_Xp[index],Adj_Xp_Avg,C_Xp_Val[index],Adj_Xp_Avg,Sensor_YpXm[index],adj_Sensor_YpXm[index],Adj_YpXm_Avg,C_YpXm_Val[index],Adj_YpXm_Avg,Sensor_YpXp[index],adj_Sensor_YpXp[index],C_YpXp_Val[index],Adj_YpXp_Avg,Sensor_Xm[index],adj_Sensor_Xm[index],C_Xm_Val[index],Adj_Xm_Avg,Theta[index],Theta_Avg,C_Theta[index],C_Theta_Avg,Theta_Std,USL_Theta,LSL_Theta,Theta_Cpk,X_axis[index],X_axis_Avg,X_axis_Std,USL_X_axis,LSL_X_axis,X_axis_Cpk,Y_axis[index],Y_axis_Avg,Y_axis_Std,USL_Y_axis,LSL_Y_axis,Y_axis_Cpk,timetoken])
		
		
					

		f_out.close()
		f_in.close()
	else:
		f_out = open(csvfilename, 'a', newline='')
		with f_out as csvfile:
			datawriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			datawriter.writerow([index,Sensor_Xp[index],adj_Sensor_Xp[index],0,0,0,Sensor_YpXm[index],adj_Sensor_YpXm[index],0,0,0,Sensor_YpXp[index],adj_Sensor_YpXp[index],0,0,Sensor_Xm[index],adj_Sensor_Xm[index],0,0,Theta[index],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,timetoken])
			f_out.close()
			f_in.close()
	index = index + 1

	try:
			Mir(IP).write_register(17,0)
	except:
			sleep(5)
			Mir(IP).write_register(17,0)



# Num of Robots
Robot_Index = 100 + int(input("Please input the number of robots in use: "))


# Initialize MiR robots
RC = 101 #Robot Counter
robot = []

while RC <= Robot_Index:
	robot.append('192.168.1.' + str(RC))
	sleep(0.5)
	RC += 1
print("IP List: ", robot)
		

# adjusts voltage readings to match read distances
sensor_read_intercept = 3.95 
sensor_read_slope = (39.5-sensor_read_intercept)/10

# Get I2C bus
try:
	bus = smbus.SMBus(1)
except:
	print("Failed to connect to MCP3426 serial manager bus")

# Setup device and sensor settings for i2c communications
address = 0x6e				#0x6e is the MCP3426 address Retrieved via cmd(sudo i2cdetect -y 1)
ch1 = [0x10,(10/8.911),"ch1"]		#[0x10(00010000) is Continuous conversion mode on Channel-1 w 12-bit Resolution, Ch1 sensor calibration value]
ch2 = [0x30,(10/8.906),"ch2"]		#[0x30(00110000) is Continuous conversion mode on Channel-2 w 12-bit Resolution, Ch2 sensor calibration value]
ch3 = [0x50,(10/8.911),"ch3"]		#[0x50(01010000) is Continuous conversion mode on Channel-3 w 12-bit Resolution, Ch3 sensor calibration value]
ch4 = [0x70,(10/8.977),"ch4"]		#[0x70(01110000) is Continuous conversion mode on Channel-4 w 12-bit Resolution, Ch4 sensor calibration value]

RC = 1
while(1):
	RC = 0
	while RC<=Robot_Index-101:
		print("Checking "+str(RC+1)+":")
		IP = robot[RC]
		print(IP)
		status, response = Mir(IP).read_register(17, timeout = 2)
		print("robot.read_register(17): ", response)
		sleep(.5)
		
		if response==1:
			print("Taking Value...")
			csvfilename = "ACC_REPEAT_DATA_MULTIPLE_MASTER_" + str(RC) + ".csv"
			Write(IP,csvfilename)
		RC += 1
