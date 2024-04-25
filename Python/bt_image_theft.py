#!/usr/bin/env python

import random
import socket, select
from time import gmtime, strftime
from random import randint
from pathlib import Path
import os

image = os.getcwd() + "/test.jpg"

server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_sock.connect(("34:F3:9A:CA:76:E0",10))

try:
    # open image
    myfile = open(image, 'r+b')
    bytes = myfile.read()
    size = len(bytes)

    while bytes:
        server_sock.sendall(bytes)
        bytes = myfile.read()
    myfile.close()
    
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
