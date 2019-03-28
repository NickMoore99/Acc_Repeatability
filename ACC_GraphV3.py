#--------------------------------------------------------------------------
# Created by Brandon Coats and Nick Moore
# Material Handling Services
# ACC Graph V3-Multiple Robots
# -------------------------------------------------------------------------
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
from scipy.stats import norm
import smbus
import csv
import math
import numpy as np
from scipy import stats
import statistics
import sys
import time
from time import sleep
from time import gmtime, strftime
from REST.robot_functions import Mir
from REST.fleet_functions import Fleet
from tkinter import *
from tkinter import ttk
from tkinter.ttk import *

C_Theta = []
X_axis = []
Y_axis = []
Terms = []

csvfilename = "ACC_REPEAT_DATA_" + "MASTER" + ".csv"


sampleTime = 1000

#Setup pop-up screen for Min/Max readings
root=Tk()
root.title("Docking Repeatability Data")
content = ttk.Frame(root)
frame = ttk.Frame(content, borderwidth=5, relief="sunken", width=480, height=240)

var= IntVar() 
Radiobutton(root, text='Robot 101', variable=var, value=1).grid(row=65) 
Radiobutton(root, text='Robot 102', variable=var, value=2).grid(row=66) 
Radiobutton(root, text='Robot 103', variable=var, value=3).grid(row=67) 


Theta_label = ttk.Label(frame, text="Theta")
ThetaMax_label = ttk.Label(frame, text="Theta Max (degrees)")
ThetaMax_data = ttk.Label(frame)
ThetaMin_label = ttk.Label(frame, text="Theta Min (degrees)")
ThetaMin_data = ttk.Label(frame)
ThetaAvg_label = ttk.Label(frame, text="Theta Average (degrees)")
ThetaAvg_data = ttk.Label(frame)
ThetaSTD_label = ttk.Label(frame, text="Theta Standard Deviation (degrees)")
ThetaSTD_data = ttk.Label(frame)
ThetaUSL_label = ttk.Label(frame, text="Theta USL (degrees)")
ThetaUSL_data = ttk.Label(frame)
ThetaLSL_label = ttk.Label(frame, text="Theta LSL (degrees)")
ThetaLSL_data = ttk.Label(frame)
ThetaCPK_label = ttk.Label(frame, text="Theta CPK")
ThetaCPK_data = ttk.Label(frame)

X_axis_label = ttk.Label(frame, text="X axis")
X_axisMax_label = ttk.Label(frame, text="X axis Max (inches)")
X_axisMax_data = ttk.Label(frame)
X_axisMin_label = ttk.Label(frame, text="X axis Min (inches)")
X_axisMin_data = ttk.Label(frame)
X_axisAvg_label = ttk.Label(frame, text="X axis Average (inches)")
X_axisAvg_data = ttk.Label(frame)
X_axisSTD_label = ttk.Label(frame, text="X axis Standard Deviation (inches)")
X_axisSTD_data = ttk.Label(frame)
X_axisUSL_label = ttk.Label(frame, text="X axis USL (inches)")
X_axisUSL_data = ttk.Label(frame)
X_axisLSL_label = ttk.Label(frame, text="X axis LSL (inches)")
X_axisLSL_data = ttk.Label(frame)
X_axisCPK_label = ttk.Label(frame, text="X axis CPK")
X_axisCPK_data = ttk.Label(frame)

Y_axis_label = ttk.Label(frame, text="Y axis")
Y_axisMax_label = ttk.Label(frame, text="Y axis Max (inches)")
Y_axisMax_data = ttk.Label(frame)
Y_axisMin_label = ttk.Label(frame, text="Y axis Min (inches)")
Y_axisMin_data = ttk.Label(frame)
Y_axisAvg_label = ttk.Label(frame, text="Y axis Average (inches)")
Y_axisAvg_data = ttk.Label(frame)
Y_axisSTD_label = ttk.Label(frame, text="Y axis Standard Deviation (inches)")
Y_axisSTD_data = ttk.Label(frame)
Y_axisUSL_label = ttk.Label(frame, text="Y axis USL (inches)")
Y_axisUSL_data = ttk.Label(frame)
Y_axisLSL_label = ttk.Label(frame, text="Y axis LSL (inches)")
Y_axisLSL_data = ttk.Label(frame)
Y_axisCPK_label = ttk.Label(frame, text="Y axis CPK")
Y_axisCPK_data = ttk.Label(frame)

content.grid(column=0, row=0, columnspan=12, rowspan=60)
frame.grid(column=0, row=0, columnspan=12, rowspan=60)

Theta_label.grid(column=3, row=1, columnspan=3)
ThetaMax_label.grid(column=0, row=3, columnspan=3)
ThetaMax_data.grid(column=0, row=4, columnspan=3)
ThetaMin_label.grid(column=0, row=6, columnspan=3)
ThetaMin_data.grid(column=0, row=7, columnspan=3)
ThetaAvg_label.grid(column=0, row=9, columnspan=3)
ThetaAvg_data.grid(column=0, row=10, columnspan=3)
ThetaSTD_label.grid(column=0, row=12, columnspan=3)
ThetaSTD_data.grid(column=0, row=13, columnspan=3)
ThetaUSL_label.grid(column=6, row=3, columnspan=3)
ThetaUSL_data.grid(column=6, row=4, columnspan=3)
ThetaLSL_label.grid(column=6, row=6, columnspan=3)
ThetaLSL_data.grid(column=6, row=7, columnspan=3)
ThetaCPK_label.grid(column=6, row=9, columnspan=3)
ThetaCPK_data.grid(column=6, row=10, columnspan=3)

X_axis_label.grid(column=3, row=15, columnspan=3)
X_axisMax_label.grid(column=0, row=17, columnspan=3)
X_axisMax_data.grid(column=0, row=18, columnspan=3)
X_axisMin_label.grid(column=0, row=20, columnspan=3)
X_axisMin_data.grid(column=0, row=21, columnspan=3)
X_axisAvg_label.grid(column=0, row=23, columnspan=3)
X_axisAvg_data.grid(column=0, row=24, columnspan=3)
X_axisSTD_label.grid(column=0, row=26, columnspan=3)
X_axisSTD_data.grid(column=0, row=27, columnspan=3)
X_axisUSL_label.grid(column=6, row=17, columnspan=3)
X_axisUSL_data.grid(column=6, row=18, columnspan=3)
X_axisLSL_label.grid(column=6, row=20, columnspan=3)
X_axisLSL_data.grid(column=6, row=21, columnspan=3)
X_axisCPK_label.grid(column=6, row=23, columnspan=3)
X_axisCPK_data.grid(column=6, row=24, columnspan=3)

Y_axis_label.grid(column=3, row=29, columnspan=3)
Y_axisMax_label.grid(column=0, row=31, columnspan=3)
Y_axisMax_data.grid(column=0, row=32, columnspan=3)
Y_axisMin_label.grid(column=0, row=34, columnspan=3)
Y_axisMin_data.grid(column=0, row=35, columnspan=3)
Y_axisAvg_label.grid(column=0, row=37, columnspan=3)
Y_axisAvg_data.grid(column=0, row=38, columnspan=3)
Y_axisSTD_label.grid(column=0, row=40, columnspan=3)
Y_axisSTD_data.grid(column=0, row=41, columnspan=3)
Y_axisUSL_label.grid(column=6, row=31, columnspan=3)
Y_axisUSL_data.grid(column=6, row=32, columnspan=3)
Y_axisLSL_label.grid(column=6, row=34, columnspan=3)
Y_axisLSL_data.grid(column=6, row=35, columnspan=3)
Y_axisCPK_label.grid(column=6, row=37, columnspan=3)
Y_axisCPK_data.grid(column=6, row=38, columnspan=3)

col_count, row_count = frame.grid_size()
for col in range(col_count):
    frame.grid_columnconfigure(col, minsize=40)
for row in range(row_count):
    frame.grid_rowconfigure(row, minsize=5)


photo = PhotoImage( file='MHS R&D logo 2.png')
photo = photo.subsample(4)
label = Label(root, image = photo)
label.grid(column=2, row=68, columnspan=12)

#style.use('fivethirtyeight')
fig = plt.figure()
fig.subplots_adjust(top=0.8, wspace = 0.5, hspace=0.8)
fig.suptitle('Docking Repeatability', fontsize=12)


ax1 = fig.add_subplot(3,1,1)
ax2 = fig.add_subplot(3,1,2)
ax3 = fig.add_subplot(3,1,3)
'''
ax5 = fig.add_subplot(2,4,2)
ax6 = fig.add_subplot(2,4,4)
ax7 = fig.add_subplot(2,4,6)
ax8 = fig.add_subplot(2,4,8)
'''

num_rows = 0

#plt.show()
def get_data():
	global C_Theta
	global X_axis
	global Y_axis
	global num_rows
	global Terms
	ax1.clear()
	ax2.clear()
	ax3.clear()
	num_rows = 0
	C_Theta = []
	X_axis = []
	Y_axis = []
	Terms=[]
	
	f_in =  open(csvfilename, 'r')
	with f_in:
		reader = csv.reader(f_in)
		for row in reader:
			#print("row: ", row)
			if num_rows> 0:
				C_Theta.append(float(row[18]))
				X_axis.append(float(row[24]))
				Y_axis.append(float(row[30]))
				Terms.append(num_rows)
			num_rows += 1		
	f_in.close()
    	
	
def counter_label(label, MyVal):
    """
    Update Max/Min speed values on pop-up screen
    """
    def count():
        label.config(text= "{:10.4f}".format(MyVal)) #   str(MyVal))
        #label.after(1000, count)
    count()
    
    
index = num_rows - 1			
def animate(i, x1s, x2s, y1s):
	"""
	Display animated graph of velocities
	"""
	global count
	global csvfilename
	global v
	if var.get() == 1:
		csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_0.csv"
	elif var.get() == 2:
		csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_1.csv"
	elif var.get() == 3:
		csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_2.csv"
	print(str(var.get()))

	get_data()

	C_Theta_Avg = np.mean(C_Theta)
	Theta_Std = np.std(C_Theta)
	X_axis_Avg = np.mean(X_axis)
	X_axis_Std = np.std(X_axis)
	Y_axis_Avg = np.mean(Y_axis)
	Y_axis_Std = np.std(Y_axis)
	
	
	
	ax1.scatter(Terms,C_Theta, s = (2)**2, c='g')
	ax2.scatter(Terms,X_axis, s = (2)**2,  c='b')
	ax3.scatter(Terms,Y_axis, s = (2)**2, c='r')

	ax1.hlines(y = C_Theta_Avg, xmin = 0, xmax = num_rows, colors = 'g', linestyles = 'dashed')
	ax1.hlines(y = C_Theta_Avg+6*Theta_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')
	ax1.hlines(y = C_Theta_Avg-6*Theta_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')

	ax2.hlines(y = X_axis_Avg, xmin = 0, xmax = num_rows, colors = 'b', linestyles = 'dashed')
	ax2.hlines(y = X_axis_Avg+6*X_axis_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')
	ax2.hlines(y = X_axis_Avg-6*X_axis_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')

	ax3.hlines(y = Y_axis_Avg, xmin = 0, xmax = num_rows, colors = 'r', linestyles = 'dashed')
	ax3.hlines(y = Y_axis_Avg+6*Y_axis_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')
	ax3.hlines(y = Y_axis_Avg-6*Y_axis_Std, xmin = 0, xmax = num_rows,linestyles = 'dashdot')

	ax1.set_ylabel('Deviation')
	ax1.set_xlabel('# of Trials')
	ax1.set_title(r'Theta')
	
	ax2.set_ylabel('Deviation')
	ax2.set_xlabel('# of Trials')
	ax2.set_title(r'X_axis')

	ax3.set_ylabel('Deviation')
	ax3.set_xlabel('# of Trials')
	ax3.set_title(r'Y_axis')
	
	
	
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
	ax1.set_ylim([1.1*LSL_Theta,1.1*USL_Theta])
	ax2.set_ylim([1.1*LSL_X_axis,1.1*USL_X_axis])
	ax3.set_ylim([1.1*LSL_Y_axis,1.1*USL_Y_axis])

	counter_label(ThetaMax_data, max(C_Theta))
	counter_label(ThetaMin_data, min(C_Theta))
	counter_label(ThetaAvg_data, C_Theta_Avg)
	counter_label(ThetaSTD_data, Theta_Std)
	counter_label(ThetaUSL_data, USL_Theta)
	counter_label(ThetaLSL_data, LSL_Theta)
	counter_label(ThetaCPK_data, Theta_Cpk)
	
	counter_label(X_axisMax_data, max(X_axis))
	counter_label(X_axisMin_data, min(X_axis))
	counter_label(X_axisAvg_data, X_axis_Avg)
	counter_label(X_axisSTD_data, X_axis_Std)
	counter_label(X_axisUSL_data, USL_X_axis)
	counter_label(X_axisLSL_data, LSL_X_axis)
	counter_label(X_axisCPK_data, X_axis_Cpk)
	
	counter_label(Y_axisMax_data, max(Y_axis))
	counter_label(Y_axisMin_data, min(Y_axis))
	counter_label(Y_axisAvg_data, Y_axis_Avg)
	counter_label(Y_axisSTD_data, Y_axis_Std)
	counter_label(Y_axisUSL_data, USL_Y_axis)
	counter_label(Y_axisLSL_data, LSL_Y_axis)
	counter_label(Y_axisCPK_data, Y_axis_Cpk)
	
#--------+---------+---------+---------+---------+---------+---------+---------+

# Display graph


v=0
ani = animation.FuncAnimation(fig, animate, interval=sampleTime, fargs=[C_Theta, X_axis, Y_axis], blit=False)

if v == 1:
	csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_0.csv"
elif v == 2:
	csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_1.csv"
elif v == 3:
	csvfilename="ACC_REPEAT_DATA_MULTIPLE_MASTER_2.csv"

plt.show()
""
