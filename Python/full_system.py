#imports
import RPi.GPIO as GPIO
import random
import socket, select
import os
import time

from pathlib import Path
from picamera2 import *
#------------------------------------------------


#network setup
port = "34:F3:9A:CA:76:E0"

server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_sock.connect((port,10))
#------------------------------------------------


#motor control setup
motor1_fwd = 12
motor1_rev = 13
motor2_fwd = 14
motor2_rev = 15

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
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
#------------------------------------------------


#image file system init
last_image_taken = 0
last_image_sent = 0
img_name = ""
image = ""
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


