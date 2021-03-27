#!/usr/bin/env python3

import socket
from test_params import PORT

#HOST = '172.17.70.161'  # The server's hostname or IP address

HOST = '127.0.0.1'  # The server's hostname or IP address
# HOST = '0.0.0.0'  # The server's hostname or IP address
# PORT = 10000    # The port used by the server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(HOST, PORT)
    s.connect((HOST, PORT))
    s.sendall(b'Hello, world')
    data = s.recv(1024)

print('Received', repr(data))