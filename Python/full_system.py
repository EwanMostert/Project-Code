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
angle = 0
spd_x = 0
spd_y = 0
pos_x = 0
pos_y = 0
#------------------------------------------------

def formatDegreesMinutes(coordinates, digits):
    parts = coordinates.split(".")
    if (len(parts) != 2):
        return coordinates
    if (digits > 3 or digits < 2):
        return coordinates
    
    left = parts[0]
    right = parts[1]
    degrees = str(left[:digits])
    minutes = str(right[:3])
    
    return (degrees + "." + minutes)

#UART pins setup
uart_tx = 8
uart_rx = 10
ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
for i in range(100):    
    dataout = pynmea2.NMEAStreamReader()
    newdata = ser.readline().decode()

    if newdata[0:10].__contains__("$GPRMC"):
        newmsg = pynmea2.parse(newdata)
        lat = newmsg.latitude
        lng = newmsg.longitude
        gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
        print(gps)
    # data = ser.readline()
    # message = data[0:6]
    
    # parts = data.decode().split(",")
    # if parts[2] == 'V':
    #     print("GPS receiver warning")
    # else:
    #     long = formatDegreesMinutes(parts[5], 3) 
    #     lat = formatDegreesMinutes(parts[3], 2)
    #     print("lon = " + str(long) + ", lat = " + str(lat))
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
#------------------------------------------------


#function to get position from GPS
def get_pos():
    rec_data = ser.read()
#------------------------------------------------