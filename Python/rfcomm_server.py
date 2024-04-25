# file: rfcomm-server.py

# auth: Albert Huang <albert@csail.mit.edu>
# desc: simple demonstration of a server application that uses RFCOMM sockets
#
# $Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $

import socket
import logging

logging.basicConfig(level=logging.INFO)

server_sock=socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server_sock.bind((socket.BDADDR_ANY,4))
server_sock.listen(20)

server_addr = server_sock.getsockname()[0]
port = server_sock.getsockname()[1]
logging.info(f"Listening on {server_addr}(port = {port})")

while True:
    client, addr = server_sock.accept()
    logging.info(f"Got new connection from {addr}")
    message = client.recv(1024).decode()
    logging.info(f"Got message from {addr}. It said '{message}'.")



