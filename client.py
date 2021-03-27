#!/usr/bin/env python3

import socket

HOST = '10.0.0.80'  # The server's hostname or IP address
from test_params import PORT

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(HOST, PORT)
    s.connect((HOST, PORT))
    s.sendall(b'Hello, world')
    data = s.recv(1024)

print('Received', repr(data)) 