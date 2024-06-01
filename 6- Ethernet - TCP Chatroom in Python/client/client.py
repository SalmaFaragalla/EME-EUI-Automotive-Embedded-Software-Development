# Author    :   Salma Faragalla
# Date      :   19/9/2023
# File      :   client.py
# Brief     :   Contains client implementation

import threading
import socket

from client_config import *

client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

def recvthread():
    while True:
        data = client.recv(1024).decode()
        if not data:
            continue
        else:
            print(f'{data}')

def sendthread ():
    while True:
        msg = input()
        if(msg == 'exit'):
            break
        else:
            client.send(msg.encode())       

def  client_start():
    client.connect((ip,port))
    threading.Thread(target=recvthread).start()
    threading.Thread(target=sendthread).start()


