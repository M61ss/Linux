#!/usr/bin/env python3
import socket
import time
import os
import json

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 8080  # Port to listen on

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    data = conn.recv(1024)
    filename = json.loads(data.decode('utf-8'))['filename']
    filesize = os.path.getsize(filename)
    file_information = json.dumps({'filename': filename, 'filesize': filesize}, indent=4, sort_keys=True)
    content = ""
    with open(filename, "r") as file:
        content = file.read()
    conn.sendall(file_information.encode('utf-8') + '\n\n'.encode('utf-8') + content.encode('utf-8'))
        
    time.sleep(1)  # socket must be closed by client; wait to avoid TIME_WAIT