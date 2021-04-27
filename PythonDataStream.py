#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC app for Data Stream from the ESP32 
Created on Sun Jan 31 10:27:57 2021

@author: kathy
"""
import socket
import pandas as pd

s = socket.socket()
s.bind(('0.0.0.0', 8090))
s.listen(0)

while True:
    
    client, addr = s.accept()
    # client handling code
    
    while True:
        content = client.recv(32)
        
        if len(content) == 0:
            break
        else:
            print(content)
            
    print("Closing connection")
    client.close()
    
