import serial
import operator
import time
import os
from functools import reduce
import numpy as np
from pathlib import Path

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

from playsound import playsound

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

thresh_dict = {20: 18,
               30: 40,
               40: 71,
               50: 111,
               60: 160,
               80: 284}

x = [1,2,3,4,5]
measurements = [0,0,0,0,0]

initialized = False
c_warning = False
c_timer = False
c_prevTime = time.perf_counter()

threshold_1 = 30
threshold_2 = 45
threshold_3 = 60

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

def validate_checksum(msg):
    data = msg.strip("$\n")
    nmea_data, checksum = data.split("*",1)
    calc_checksum = reduce(operator.xor,(ord(s) for s in nmea_data), 0)
    if int(checksum, base=10) == calc_checksum:
        return nmea_data
    else:
        print(checksum)
        print(calc_checksum)
        raise ValueError("The NMEA data does not match it's checksum")

def calc_slope(x,y):
    A = np.vstack([x, np.ones(len(x))]).T
    m, c = np.linalg.lstsq(A.astype('float'), y.astype('float'), rcond=None)[0]
    return m, c 

def parse_message(msg):
    data = validate_checksum(msg)
    header = data.split(",")[0]
    if(header == "HDIST"):
        # add measurement to array
        measurements.pop(0)
        measurements.extend([data.split(",")[1]])
        distanceString.set("Distance (cm): {:0.1f}".format(float(data.split(",")[1])))
        #print(measurements)
    elif(header == "COLIS"):
        if(data.split(",")[1] == 1):
            print("Collision true")
            

def collision_warning():
    global c_prevTime, c_timer, c_warning
    if(time.perf_counter() - c_prevTime > 2):
        c_timer = True
    if(c_timer == True & c_warning == True):
        collisionString.set("Collision warning!")
        c_timer = False
        c_prevTime = time.perf_counter()
    elif(c_timer == True & c_warning == False):
        collisionString.set("")

            
def collision_detect():
    global c_warning
    m,c = calc_slope(np.array(x),np.array(measurements))
    if(all(float(x) < threshold_1 for x in measurements[2:4])):
        # Red region
        if(m < -0.5):
            # Collision
            image_label.config(image=small_red_img)
            c_warning = True
        elif(m >= -0.5):
            image_label.config(image=small_red_img)
            c_warning = False
    elif(all(float(x) >= threshold_1 for x in measurements[2:4]) & all(float(x) < threshold_2 for x in measurements[2:4])):
        # Yellow region
        if(m < -1):
            image_label.config(image=small_yellow_img)
            c_warning = True
        elif(m > -1):
            image_label.config(image=small_yellow_img)
            c_warning = False
    elif(all(float(x) >= threshold_2 for x in measurements[2:4]) & all(float(x) < threshold_3 for x in measurements[2:4])):
        # Green region
        image_label.config(image=small_green_img)
        c_warning = False
    elif(all(float(x) >= threshold_3 for x in measurements[2:4])):
        # Blue region
        image_label.config(image=small_blue_img)
        c_warning = False

def thresh_criteria(dict_item):
    key, value = dict_item
    return abs(key - user_speed)

def speed_callback():
    global user_speed, threshold_1, threshold_2, threshold_3
    user_speed = int(speedString.get())
    threshold_1 = float(min(thresh_dict.items(), key=thresh_criteria)[1])*0.1
    threshold_2 = threshold_1*2
    threshold_3 = threshold_1*2.5
    msg = "$SPEED," + speedString.get() + "," + str(threshold_1) + "\n"
    ser.write(msg.encode())
    resultString.set("Speed (mph): {}".format(speedString.get()))
    thresh1String.set("Red Threshold (cm): {:0.1f}".format(threshold_1))
    thresh2String.set("Yellow Threshold (cm): {:0.1f}".format(threshold_2))
    thresh3String.set("Green Threshold (cm): {:0.1f}".format(threshold_3))
    image_label.config(image=small_blue_img)
    global initialized
    initialized = True

def plot_callback():
    #plt.ion()
    figure, ax = plt.subplots()
    ax.plot(x_plot.astype(float),y_plot.astype(float))
    ax.set_ylim([0, 100])
    plt.autoscale(enable=True, axis='x')
    plt.show()
    

ser = serial.Serial('COM3', 115200, timeout=5)

#speed = input('Enter initial speed: ')

#ser.write(speed.encode())

rl = ReadLine(ser)

# GUI
app = tk.Tk()
app.geometry("600x400")
app.title('Collision Detection System')

labelSpeed = tk.Label(app, text="Speed (mph)")
labelSpeed.grid(column=0, row=0, sticky=tk.W)

speedString = tk.StringVar()
entrySpeed = tk.Entry(app, width=20, textvariable=speedString)

entrySpeed.grid(column=1, row=0, padx=10)

resultButton = tk.Button(app, text="Send Speed", command=speed_callback)

resultButton.grid(column=0, row=1, pady=10, sticky=tk.W)

resultString = tk.StringVar()
resultString.set("Speed (mph): 0")
resultLabel = tk.Label(app, textvariable=resultString)
resultLabel.grid(column=0, row=2, columnspan=2, padx=10, sticky=tk.W)

thresh1String = tk.StringVar()
thresh1String.set("Red Threshold (cm): 0")
thresh1Label = tk.Label(app, textvariable=thresh1String)
thresh1Label.grid(column=0, row=3, columnspan=2, padx=10, sticky=tk.W)

thresh2String = tk.StringVar()
thresh2String.set("Yellow Threshold (cm): 0")
thresh2Label = tk.Label(app, textvariable=thresh2String)
thresh2Label.grid(column=0, row=4, columnspan=2, padx=10, sticky=tk.W)

thresh3String = tk.StringVar()
thresh3String.set("Green Threshold (cm): 0")
thresh3Label = tk.Label(app, textvariable=thresh3String)
thresh3Label.grid(column=0, row=5, columnspan=2, padx=10, sticky=tk.W)

collisionString = tk.StringVar()
collisionLabel = tk.Label(app, textvariable=collisionString, font=('Arial',25))
collisionLabel.grid(column=2, row=10, columnspan=6, padx=10, sticky=tk.W)

distanceString = tk.StringVar()
distanceString.set("Distance (cm): 0")
distanceLabel = tk.Label(app, textvariable=distanceString)
distanceLabel.grid(column=0, row=6, padx=10, sticky=tk.W)

plotButton = tk.Button(app, text="Plot", command=plot_callback)
plotButton.grid(column=0, row=7, pady=10, sticky=tk.W)

img = tk.PhotoImage(file = r"uninitialized.png")
img1 = img.subsample(8, 8)
image_label = tk.Label(app, image = img1)
image_label.grid(row = 0, column = 2,
       columnspan = 6, rowspan = 8, padx = 5, pady = 5)

blue_img = tk.PhotoImage(file = r"blue.png")
small_blue_img = blue_img.subsample(8, 8)

green_img = tk.PhotoImage(file = r"warning_green.png")
small_green_img = green_img.subsample(8, 8)

yellow_img = tk.PhotoImage(file = r"warning_yellow.png")
small_yellow_img = yellow_img.subsample(8, 8)

red_img = tk.PhotoImage(file = r"warning_red.png")
small_red_img = red_img.subsample(8, 8)

#app.mainloop()

idx = 0
x_plot = np.array([0])
y_plot = np.array([0])

#plt.ion()  # Interactive mode on
#figure, ax = plt.subplots(figsize=(10, 8))

#def animate_graph(i,x_plot,y_plot):
#    x_plot.append(time.perf_counter())
#    y_plot.append(measurements[4])
#
#    x_plot = x_plot[-50:]
#    y_plot = y_plot[-50:]
#
#    ax.clear()
#    ax.plot(x_plot, y_plot)
#
#    plt.subplots_adjust(bottom=0.30)
#    plt.title('HC-SR04 Ultrasonic Sensor vs. Time')
#    plt.ylabel('Distance (cm)')

#ani = FuncAnimation(figure, animate_graph, fargs=(x_plot, y_plot), interval=1000)

while True:

    app.update()

    if(initialized == True):

        idx += 1
        message = rl.readline().decode()
    
        parse_message(message)

        #m,c = calc_slope(np.array(x),np.array(measurements))
        collision_detect()
        collision_warning()

        x_plot = np.append(x_plot,time.perf_counter())
        y_plot = np.append(y_plot,measurements[4])

        #line.set_xdata(x_plot.append(idx))
        #line.set_ydata(y_plot.append(measurements[4]))
        #figure.canvas.draw()
        #figure.canvas.flush_events()
        #figure.gca().relim()
        #figure.gca().autoscale_view()

        #plt.pause(0.001)

        print(message)

    time.sleep(0.01)
