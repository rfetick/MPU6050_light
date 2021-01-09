# -*- coding: utf-8 -*-
"""
Created on Sat Mar 14 13:24:50 2020

Dynamic plot of the MPU6050 angles. How to use:
    - Upload the 'getAngle.ino' code to the Arduino
    - Modify the port name below according to your own port
    - Run this script

Requires the Python libraries: serial, matplotlib, numpy

@author: rfetick
"""

from serial import Serial
import matplotlib.pyplot as plt
import numpy as np
import re

#%% DEFINITIONS
port = "COM7"   # modify this port name !!!
baudrate = 9600 # must be similar to the one in 'getAngle.ino'
N = 200         # number of plotted measurements
M = 2000        # number of total measurements before stop

#%% DYNAMIC PLOT
# https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
class DynamicUpdate():
    def on_launch(self):
        self.figure, self.ax = plt.subplots()
        self.ax.axes.set_xlabel('Iteration')
        self.ax.axes.set_ylabel('Angle [deg]')
        self.ax.plot([],[],'o')
        self.ax.plot([],[],'o')
        self.ax.plot([],[],'o')
        self.ax.set_ylim(-180,180)
        self.ax.set_xlim(0, N)
        self.ax.grid()
        T = np.arange(N,dtype=float)
        for k in range(3):
            self.ax.lines[k].set_xdata(T)
            self.ax.lines[k].set_ydata(0*T)

    def on_running(self, y, y2, y3):
        self.ax.lines[0].set_ydata(y)
        self.ax.lines[1].set_ydata(y2)
        self.ax.lines[2].set_ydata(y3)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

du = DynamicUpdate()
du.on_launch()

#%% READ SERIAL
DATA = np.zeros((N,3))

with Serial(port=port, baudrate=baudrate, timeout=1, writeTimeout=1) as port_serie:
    if port_serie.isOpen():
        port_serie.flush()
        print('Serial port open: wait for first data')
        for i in range(8): # do not process first lines
            ligne = port_serie.readline()
            print(str(ligne)[2:-1])
        for i in range(M):
            ligne = port_serie.readline()
            j = i * (i<N) + (N-1) * (i>=N)
            if i>=N: DATA = np.roll(DATA,-1,axis=0)
            try:
                temp = re.findall('[-.0-9]+',str(ligne)[2:-5])
                DATA[j,:] = [float(t) for t in temp]
                if (i%5)==0:
                    du.on_running(DATA[:,0],DATA[:,1],DATA[:,2])
            except:
                print("Exception: %s"%ligne)
        port_serie.close()
    else:
        print('Warning: could not open serial port '+port)