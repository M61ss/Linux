#!/usr/bin/env python3
import socket
import sys
import json
import os

HOST = sys.argv[1]
PORT = int(sys.argv[2])

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    filename = input('Please enter the file you want to download: ')
    s.sendall(json.dumps({'filename': filename}, indent=4, sort_keys=True).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    header, content = data.split('\n\n', 1)
    header = json.loads(header)
    filename = header['filename']
    filesize = header['filesize']
    with open(filename, "w") as file:
        file.write(content)
        bytes_remaining = filesize - len(content)
        while bytes_remaining > 0:
            more_content = s.recv(min(1024, bytes_remaining))
            if not more_content:
                break
            file.write(more_content)
            bytes_remaining -= len(more_content)
    s.close()
