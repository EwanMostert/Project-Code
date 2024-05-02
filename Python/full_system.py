#imports
import RPi.GPIO as GPIO
import random
import socket, select
import os
import time
import serial
import string
import pynmea2

from gps import *
from pathlib import Path
from picamera2 import *
#------------------------------------------------


#System setup:
running = True
start_time = time.time()
#------------------------------------------------


#network setup
# port = "34:F3:9A:CA:76:E0"

# server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
# server_sock.connect((port,10))
#------------------------------------------------


#motor control setup
# motor1_fwd = 22
# motor1_rev = 24
# motor2_fwd = 26
# motor2_rev = 28

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
pos_x = 0
pos_y = 0
angle = 0
spd_x = 0
spd_y = 0

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
        pos_y = newmsg.latitude
        pos_x = newmsg.longitude
        gps = "Latitude =" + str(pos_y) + " and Longitude =" + str(pos_x)
        print(gps)
#------------------------------------------------


#Calculate heading and speed:
def get_angle():
    return

def get_speed():
    return

#------------------------------------------------


#Main system loop:
    while running == True:
        get_pos()
#------------------------------------------------