#imports
import RPi.GPIO as GPIO
import random
import socket, select
import os
import time
import serial
import string
import pynmea2
import threading
import logging

from gpiozero import PWMOutputDevice
from math import *
from gps import *
from pathlib import Path
from picamera2 import *
#------------------------------------------------


#network setup
# port = "5C:80:B6:30:A9:0C"
port = "34:F3:9A:CA:76:E0"
bt_connected = False
server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

def try_bt_connect(bt_running):
    global server_sock
    global bt_connected
    while bt_running.is_set():
        if bt_connected == False:            
            try:
                server_sock.connect((port,10))
                print("Connected to server")
                bt_connected = True
            except:
                print("Not connected to server")
                server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)               
        time.sleep(10.0)   
    return 
#------------------------------------------------


#motor control setup
motor1_fwd = PWMOutputDevice(25)
motor1_rev = PWMOutputDevice(8)
motor2_fwd = PWMOutputDevice(7)
motor2_rev = PWMOutputDevice(1)

motor1_fwd.frequency = 10000
motor1_rev.frequency = 10000
motor2_fwd.frequency = 10000
motor2_rev.frequency = 10000

D = 0
d_1 = 0
d_2 = 0

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)

# GPIO.setup(motor1_fwd,GPIO.OUT)
# motor1_fwd_pwm = GPIO.PWM(motor1_fwd,10000)
# GPIO.setup(motor1_rev,GPIO.OUT)
# motor1_rev_pwm = GPIO.PWM(motor1_rev,10000)

# GPIO.setup(motor2_fwd,GPIO.OUT)
# motor2_fwd_pwm = GPIO.PWM(motor2_fwd,10000)
# GPIO.setup(motor2_rev,GPIO.OUT)
# motor2_rev_pwm = GPIO.PWM(motor2_rev,10000)
#------------------------------------------------


#image file system init
last_image_taken = 0
last_image_sent = 0
img_name = ""
image = ""
#------------------------------------------------


#camera setup
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
#------------------------------------------------


#positional data init
ROTATION_PERIOD = 10

pos_x_new = 0
pos_y_new = 0
pos_x_old = 0
pos_y_old = 0
pos_change = 0

start_y = -33.929687
start_x = 18.85424

goal_y = -33.930015
goal_x = 18.855235
goal_angle = -90

speed = 0
angle = -90
alpha = 0

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
except Exception as error:
    print("Could not establish serial connection", error)
    uart_connected = False  
#------------------------------------------------


#GPS test
# count = 0
# time_1 = 0
# time_2 = 0
# on_time = False
# for i in range(100):        
#     dataout = pynmea2.NMEAStreamReader()
#     try:                        
#         newdata = ser.readline().decode()
#         if newdata[0:10].__contains__("$GPRMC"):
#             if on_time == False:
#                 time_1 = time.time()
#                 on_time = True
#             else:
#                 time_2 = time.time()
#                 on_time = False    
#             newmsg = pynmea2.parse(newdata)
#             lat = newmsg.latitude
#             lng = newmsg.longitude
#             gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
#             print(gps)
#             count += 1
#     except:
#         print("Could not read from serial channel")
#     print(time_2 - time_1)
#------------------------------------------------


#function for taking photo
def take_photo():
    global last_image_taken
    global img_name
    global picam2
    img_name = "image" + str(last_image_taken) + ".jpg"
    picam2.start()
    picam2.capture_file(img_name)
    last_image_taken += 1
    return   
#------------------------------------------------


#function for sending photo
def send_photo():
    global last_image_sent
    global img_name
    global server_sock
    global bt_connected
    try:
    #open image        
        img_name = "image" + str(last_image_sent) + ".jpg"
        image = os.getcwd() + "/" + img_name
        myfile = open(image, 'r+b')
        bytes = myfile.read()
        size = len(bytes)

    #request to send data
        message = (b'sending data')
        print("Raising hand")
        server_sock.send(message)
        answer = server_sock.recv(1024).decode()
        if answer.__contains__("go ahead"):
            print("Begin data send")
            #send image
            while bytes:
                server_sock.sendall(bytes)
                bytes = myfile.read()
            myfile.close()
            last_image_sent += 1
            bt_connected = False

    finally:
        server_sock.close()
        print("Disconnected from server")
        bt_connected = False
    return

#BLUETOOTH PROTOCOL:
#After initial connection of the socket, the communication protocol between server (PC) and client (RPi) will be as follows:
#Client will request to send data
#Server will acknowledge request
#Client will begin sending image
#Server will begin receiving image
#------------------------------------------------


#function to get position from GPS
def get_pos():
    global uart_connected
    global ser
    global pos_x_new, pos_y_new
    global pos_x_old, pos_y_old
    global time_new, time_old
    global logger
    
    if uart_connected == False:
        try:
            ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
            dataout = pynmea2.NMEAStreamReader()
            uart_connected = True        
            logger.debug("UART connected")
        except Exception as error:
            logger.debug(error)
            uart_connected = False    
    else:
        try:
            newdata = ser.readline().decode()    
            if newdata.__contains__("$GPRMC"):
                newmsg = pynmea2.parse(newdata)
                pos_x_old = pos_x_new
                pos_y_old = pos_y_new
                time_old = time_new
                time_new = time.time()
                pos_y_new = newmsg.latitude
                pos_x_new = newmsg.longitude
                gps = "Latitude =" + str(pos_y_new) + " and Longitude =" + str(pos_x_new)
                # print(gps)
        except Exception as error:
            logger.debug(error)
            uart_connected = False
#------------------------------------------------


#Calculate heading and speed and distance:
def calc_distance(pos_x_2,pos_y_2,pos_x_1,pos_y_1):
    angle_x = pos_x_2 - pos_x_1
    angle_y = pos_y_2 - pos_y_1
    angle_change = radians(sqrt(pow(angle_x,2) + pow(angle_y,2)))
    result = angle_change * 6371 * 1000
    return result

def calc_speed():
    global speed
    global pos_change
    global time_new, time_old
    if time_new == 0:
        speed = 0
    else:
        speed = pos_change / (time_new - time_old)

def calc_angle(pos_x_2,pos_y_2,pos_x_1,pos_y_1):
    result = degrees(atan((pos_x_2 - pos_x_1)/(pos_y_2 - pos_y_1)))
    return result

def calc_error():
    global dist_err, angle_err
    global angle
    global goal_x,goal_y,pos_x_new,pos_y_new
    global alpha
    
    dist_err = calc_distance(goal_x,goal_y,pos_x_new,pos_y_new)
    angle_err = angle - calc_angle(goal_x,goal_y,pos_x_new,pos_y_new)
    a = sin(radians(angle_err)) / (1 - cos(radians(angle_err)))
    alpha = degrees(atan(a))
#------------------------------------------------


#Actuation functions:
def set_motorspeed():
    global dist_err
    global d_1, d_2
    if dist_err <= 10:
        if d_1 > 0.5:
            d_1 = 0.5
        if d_2 > 0.5:
            d_2 = 0.5
    
def ramp_up():
    global D
    global d_1, d_2
    for i in range(0, 101, 1):
        D = i / 100
        d_1 = D
        d_2 = D
        activate_motors()

def ramp_down():
    global D
    global d_1, d_2
    for i in range(100, -1, -1):
        D = i / 100
        d_1 = D
        d_2 = D
        activate_motors()

def set_motordirection():
    global d_1, d_2
    global alpha
    
    if alpha > 0:
        alpha = 90 - alpha
    if alpha < 0:
        alpha = -90 - alpha
        
    chng_rate = sin(radians(alpha))
    
    if alpha < 0:
        d_1 = 1 + chng_rate
        d_2 = 1
    if alpha > 0:
        d_1 = 1
        d_2 = 1 - chng_rate

def rotate():
    global goal_angle
    global angle
    global d_1, d_2
    global ROTATION_PERIOD
    angle_change = goal_angle - angle
    if angle_change > 180:
        angle_change -= 360
        if angle_change > 0:
            d_1 = -0.5
            d_2 = 0.5
        else:
            d_1 = 0.5
            d_2 = -0.5
    activate_motors()
    sleep_time = angle_change / 360 * ROTATION_PERIOD
    time.sleep(sleep_time)
    d_1 = 0
    d_2 = 0
    activate_motors()

def activate_motors():
    global motor1_fwd, motor1_rev
    global motor2_fwd, motor2_rev
    global d_1, d_2
    if d_1 > 0:
        motor1_fwd.value = d_1
        motor1_rev.value = 0
        # motor1_fwd_pwm.ChangeDutyCycle(d_1)
        # motor1_rev_pwm.ChangeDutyCycle(0)
    elif d_1 < 0:
        motor1_fwd.value = 0
        motor1_rev.value = -1*d_1
        # motor1_fwd_pwm.ChangeDutyCycle(0)
        # motor1_rev_pwm.ChangeDutyCycle(-1*d_1)

    if d_2 > 0:
        motor2_fwd.value = d_2
        motor2_rev.value = 0
        # motor2_fwd_pwm.ChangeDutyCycle(d_2)
        # motor2_rev_pwm.ChangeDutyCycle(0)
    elif d_2 < 0:
        motor2_fwd.value = 0
        motor2_rev.value = -1*d_2
        # motor2_fwd_pwm.ChangeDutyCycle(0)
        # motor2_rev_pwm.ChangeDutyCycle(-1*d_2)    

#------------------------------------------------


#Logging functionality:
def do_logging(log_running):
    global logger
    while log_running.is_set():
        log_msg = generate_log_msg()
        logger.debug(log_msg)
        time.sleep(1.0)
    return

def generate_log_msg():
    global pos_x_new, pos_y_new
    global speed, angle, alpha
    global d_1, d_2
    global dist_err, angle_err
    
    msg = str(round(pos_y_new,5)) + "," + str(round(pos_x_new,5)) + "," + str(round(speed,3)) + "," + str(round(angle)) + "," + str(dist_err) + "," + str(round(angle_err)) + "," + str(round(alpha,2)) + "," + str(d_1) + "," + str(d_2)
    
    return msg
#------------------------------------------------


#System setup:
running = True
in_transit = False
at_goal = False
pos_x_new = start_x
pos_y_new = start_y
pos_x_old = start_x
pos_y_old = start_y
calc_error()
start_time = time.time()
time_new = start_time
time_old = start_time
#------------------------------------------------


#Main system loop:
bt_running = threading.Event()
bt_running.set()
bt_thread = threading.Thread(target=try_bt_connect, args=(bt_running,))
bt_thread.start()

logging.basicConfig(filename="/home/proj448/Documents/GitHub/Project-Code/Python/logging.txt",format='%(asctime)s %(message)s',filemode='a',level=logging.DEBUG)
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

log_running = threading.Event()
log_running.set()
log_thread = threading.Thread(target=do_logging, args=(log_running,))
log_thread.start()

in_transit = True
ramp_up()

while (running == True):
    get_pos()
    # print("I am at" + str(pos_x_new) + ", " + str(pos_y_new))
    # print("Goal is at" + str(goal_x) + ", " + str(goal_y))
    if in_transit == True:
        if calc_distance(pos_x_new,pos_y_new,pos_x_old,pos_y_old) > 2.5:
            pos_change = calc_distance(pos_x_new,pos_y_new,pos_x_old,pos_y_old)
            calc_speed()
            angle = calc_angle(pos_x_new,pos_y_new,pos_x_old,pos_y_old)
            calc_error()  
            
            set_motordirection()
            set_motorspeed()   
            activate_motors()       
            if dist_err <= 3:
                in_transit = False
                ramp_down() 
                
    if in_transit == False and at_goal == False:
        rotate()
        calc_error()
        if angle_err <= 10:
            at_goal = True

    # if bt_connected == True and at_goal == True:
    if at_goal == True and bt_connected == True:
        take_photo()
        send_photo()
        running = False

bt_running.clear()  
log_running.clear()    
#------------------------------------------------