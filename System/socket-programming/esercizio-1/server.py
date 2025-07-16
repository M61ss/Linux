#!/usr/bin/env python3
import socket
import sys
import os
import time

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 2525  # Port to listen on

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        pid = os.fork()
        if pid < 0:
            print("Fork failed")
        elif pid == 0:
            print("Process " + str(os.getpid()))
            conn.sendall(("Welcome from " + socket.gethostname()).encode('utf-8'))
            time.sleep(1)  # socket must be closed by client; wait to avoid TIME_WAIT