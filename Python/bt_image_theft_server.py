#!/usr/bin/env python

import random
import socket, select
from time import gmtime, strftime
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
used = 1

while True:
    client, addr = server_sock.accept()
    client.setblocking(True)
    logging.info(f"Got new connection from {addr}")
    
    try:
        myfile = open(basename % imgcounter, 'w+b')
        data = client.recv(8388608)
        while data :
            myfile.write(data)
            data = client.recv(8388608)
        myfile.close()
        print("hello")
    #     if used == 1:
    #         # print(f"Buffer size is {buffer_size}")
    #         data = client.recv(4096)
    #         txt = str(data)
    #         used = 0

    #     if data:
    #         if txt.__contains__('SIZE'):
    #             # print(f"{txt}")
    #             txt = txt.replace("'","")
    #             # print(txt)
    #             tmp = txt.split()
    #             # print(tmp)
    #             size = int(tmp[1])

    #             # print("got size")
    #             # print("size is %s" % size)

    #             client.sendall("GOT SIZE".encode())
    #             # Now set the buffer size for the image 
    #             data = None

    #         elif txt.__contains__('BYE'):
    #             client.shutdown()

    #     else:
    #         print("hello")
    #         myfile = open(basename % imgcounter, 'wb')
    #         myfile.write(data)

    #         data = client.recv(size)
    #         if not data:
    #             myfile.close()
    #             break
    #         myfile.write(data)
    #         myfile.close()

    #         client.sendall("GOT IMAGE".encode())
    #         client.shutdown()    
    except: 
        print("hello")
        client.close()
        break
        continue

    # imgcounter += 1

server_sock.close()