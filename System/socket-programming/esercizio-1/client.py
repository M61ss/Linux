#!/usr/bin/env python3
import socket
import sys

HOST = '127.0.0.1'
PORT = int(sys.argv[1])

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    data = s.recv(1024)
    s.close()

print('Received: %s' % data.decode('utf-8'))