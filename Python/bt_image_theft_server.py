#!/usr/bin/env python

import random
import socket, select
from time import gmtime, strftime, sleep
from random import randint
import logging

imgcounter = 1
basename = "image%s.jpg"

logging.basicConfig(level=logging.INFO)

server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_sock.bind((socket.BDADDR_ANY,10))
server_sock.listen(2)

server_addr = server_sock.getsockname()[0]
port = server_sock.getsockname()[1]
logging.info(f"Listening on {server_addr}(port = {port})")
connected = False

while True:
    if connected == False:
        client, addr = server_sock.accept()
        client.setblocking(True)
        logging.info(f"Got new connection from {addr}")
        connected = True
    
    if connected == True and addr != None:
        try:
            data = client.recv(1024)
            if data != None:
                myfile = open(basename % imgcounter, 'w+b')
                imgcounter += 1
                txt = data.decode()
            if txt.__contains__("sending data"):
                message = (b'go ahead')
                client.send(message)
                data = client.recv(4096)
                while data :
                    myfile.write(data)
                    data = client.recv(4096)
                myfile.close()
                print("File received")
            client.detach()
            addr = None
            connected = False
            
  
        except:
            print("Client error")
            client.detach()
            addr = None
            connected = False
    data = None
    txt = ""
    sleep(2.0)