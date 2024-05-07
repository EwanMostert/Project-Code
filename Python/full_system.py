#imports
import RPi.GPIO as GPIO
import random
import socket, select
import os
import time
import serial
import string
import pynmea2
from math import *

from gps import *
from pathlib import Path
from picamera2 import *
#------------------------------------------------


#System setup:
running = True
start_time = time.time()
#------------------------------------------------


#network setup
port = "34:F3:9A:CA:76:E0"
bt_timeout = 0
#------------------------------------------------


#motor control setup
motor1_fwd = 22
motor1_rev = 24
motor2_fwd = 26
motor2_rev = 28

D = 0
d_1 = 0
d_2 = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(motor1_fwd,GPIO.OUT)
motor1_fwd_pwm = GPIO.PWM(motor1_fwd,10000)
GPIO.setup(motor1_rev,GPIO.OUT)
motor1_rev_pwm = GPIO.PWM(motor1_rev,10000)

GPIO.setup(motor2_fwd,GPIO.OUT)
motor2_fwd_pwm = GPIO.PWM(motor2_fwd,10000)
GPIO.setup(motor2_rev,GPIO.OUT)
motor2_rev_pwm = GPIO.PWM(motor2_rev,10000)
#------------------------------------------------


#camera setup
# picam2 = Picamera2()
# camera_config = picam2.create_preview_configuration()
# picam2.configure(camera_config)
#------------------------------------------------


#image file system init
last_image_taken = 0
last_image_sent = 0
img_name = ""
image = ""
#------------------------------------------------


#positional data init
ROTATION_PERIOD = 10

pos_x_new = 0
pos_y_new = 0
pos_x_old = 0
pos_y_old = 0
pos_change = 0

start_x = 0
start_y = 0

goal_x = 0
goal_y = 0
goal_angle = 0

speed = 0
angle = 90

dist_err = 0
angle_err = 0

time_new = 0
time_old = 0

#NOTE: Latitude will be pos_y and longitude will be pos_x. Speed and angle will be calculated from changes in position.
#------------------------------------------------


#UART setup
uart_tx = 8
uart_rx = 10
uart_connected = False
try:
    ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
    dataout = pynmea2.NMEAStreamReader()
    uart_connected = True
except:
    print("Could not establish serial connection")    
#------------------------------------------------


#GPS test
count = 0
time_1 = 0
time_2 = 0
on_time = False
for i in range(100):        
    dataout = pynmea2.NMEAStreamReader()
    try:                        
        newdata = ser.readline().decode()
        if newdata[0:10].__contains__("$GPRMC"):
            if on_time == False:
                time_1 = time.time()
                on_time = True
            else:
                time_2 = time.time()
                on_time = False    
            newmsg = pynmea2.parse(newdata)
            lat = newmsg.latitude
            lng = newmsg.longitude
            gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
            print(gps)
            count += 1
    except:
        print("Could not read from serial channel")
    print(time_2 - time_1)
#------------------------------------------------


#function for taking photo
def take_photo():
    img_name = "image" + str(last_image_taken) + ".jpg"
    picam2.start_preview(Preview.QTGL)
    picam2.start()
    time.sleep(2)
    picam2.capture_file(img_name)
    last_image_taken += 1    
#------------------------------------------------


#function for sending photo
def send_photo():
    try:
    # open image
        img_name = "image" + str(last_image_sent) + ".jpg"
        image = os.getcwd() + img_name
        myfile = open(image, 'r+b')
        bytes = myfile.read()
        size = len(bytes)

    #send image
        while bytes:
            server_sock.sendall(bytes)
            bytes = myfile.read()
        myfile.close()
        last_image_sent += 1
    
    # # send image size to server
    # server_sock.sendall(("SIZE %s" % size).encode())
    # answer = server_sock.recv(4096)

    # print(f"answer = {answer}")

    # # send image to server
    # txt = str(answer)
    # if txt.__contains__('GOT SIZE'):        
    #     server_sock.sendall(bytes)
    #     print("hello")

    #     # check what server send
    #     answer = server_sock.recv(4096)
    #     txt = str(answer)
    #     print('answer = %s' % answer)

    #     if txt.__contains__('GOT IMAGE'):
    #         server_sock.sendall("BYE BYE ".encode())
    #         print('Image successfully send to server')

    # myfile.close()

    finally:
        server_sock.close()

#BLUETOOTH PROTOCOL:
#After initial connection of the socket, the communication protocol between server (PC) and client (RPi) will be as follows:
#Client will request to send data
#Server will acknowledge request
#Client will begin sending image
#Server will begin receiving image
#------------------------------------------------


#function to get position from GPS
def get_pos():
    if uart_connected == False:
        try:
            ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
            dataout = pynmea2.NMEAStreamReader()
            uart_connected = True
        except:
            print("Could not establish serial connection") 
            return 
    try:
        newdata = ser.readline().decode()
    except:
        print("Could not read from serial channel")
        return

    if newdata[0:10].__contains__("$GPRMC"):
        newmsg = pynmea2.parse(newdata)
        pos_x_old = pos_x_new
        pos_y_old = pos_y_new
        pos_y_new = newmsg.latitude
        pos_x_new = newmsg.longitude
        gps = "Latitude =" + str(pos_y_new) + " and Longitude =" + str(pos_x_new)
        print(gps)
#------------------------------------------------


#Calculate heading and speed and distance:
def calc_distance(pos_x_2,pos_y_2,pos_x_1,pos_y_1):
    angle_x = pos_x_2 - pos_x_1
    angle_y = pos_y_2 - pos_y_1
    angle_change = radians(sqrt(pow(angle_x,2) + pow(angle_y,2)))
    result = angle_change * 6371 * 1000
    return result

def calc_speed():
    speed = pos_change / (time_new - time_old)

def calc_angle(pos_x_2,pos_y_2,pos_x_1,pos_y_1):
    result = degrees(atan((pos_x_2 - pos_x_1)/(pos_y_2 - pos_y_1)))
    return result

def calc_error():
    dist_err = calc_distance(goal_x,goal_y,pos_x_new,pos_y_new)
    angle_err = angle - calc_angle(goal_x,goal_y,pos_x_new,pos_y_new)

#------------------------------------------------


#Actuation functions:
def set_motorspeed():
    if dist_err > 5:
        D = 100
    else:
        D = 50
    d_1 *= D
    d_2 *= D

def ramp_up():
    for i in range(100,-1,-1):
        D = i
        d_1 = D
        d_2 = D
        activate_motors()


def ramp_down():
    for i in range(100,-1,-1):
        D = i
        d_1 = D
        d_2 = D
        activate_motors()

def set_motordirection():
    d_1 = cos(angle_err)
    d_2 = sin(angle_err)

def rotate():
    angle_change = goal_angle - angle
    if angle_change > 180:
        angle_change -= 360
        if angle_change > 0:
            d_1 = -50
            d_2 = 50
        else:
            d_1 = 50
            d_2 = -50
    activate_motors()
    sleep_time = angle_change / 360 * ROTATION_PERIOD
    time.sleep(sleep_time)
    d_1 = 0
    d_2 = 0
    activate_motors()


def activate_motors():
    if d_1 > 0:
        motor1_fwd_pwm.ChangeDutyCycle(d_1)
        motor1_rev_pwm.ChangeDutyCycle(0)
    elif d_1 < 0:
        motor1_fwd_pwm.ChangeDutyCycle(0)
        motor1_rev_pwm.ChangeDutyCycle(-1*d_1)

    if d_2 > 0:
        motor2_fwd_pwm.ChangeDutyCycle(d_2)
        motor2_rev_pwm.ChangeDutyCycle(0)
    elif d_2 < 0:
        motor2_fwd_pwm.ChangeDutyCycle(0)
        motor2_rev_pwm.ChangeDutyCycle(-1*d_2)
    return    

#------------------------------------------------


#Main system loop:
    while running == True:
        if bt_timeout >= 
        try:
            server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            server_sock.connect((port,10))
        except:
            print("Could not connect bluetooth")
        get_pos()
        if calc_distance(pos_x_new,pos_y_new,pos_x_old,pos_y_old) >= 2.5:
            pos_change = calc_distance(pos_x_new,pos_y_new,pos_x_old,pos_y_old)
            calc_speed()
            angle = calc_angle(pos_x_new,pos_y_new,pos_x_old,pos_y_old)
#------------------------------------------------