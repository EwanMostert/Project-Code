# file: rfcomm-server.py

# auth: Albert Huang <albert@csail.mit.edu>
# desc: simple demonstration of a server application that uses RFCOMM sockets
#
# $Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $

import socket
import logging

logging.basicConfig(level=logging.INFO)

server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_sock.connect(("34:F3:9A:CA:76:E0",4))
server_sock.send("hello from client".encode())
    


