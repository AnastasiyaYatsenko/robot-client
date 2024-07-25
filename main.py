#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
from time import sleep
from typing import NamedTuple
from struct import *
import threading
import sys

from hand import setupGPIO
from server import Robot


robot = Robot()

MAX_CONNECTIONS = 10
address_to_server = ('192.168.0.91', 8686)

# конектимось до сервера
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(address_to_server)


def send_params():
    #for i in range(3):
    #    robot.get()
    robot.get()
    p = pack('@ffiffiffi',
             robot.get_results[0][4], robot.get_results[0][5], robot.get_results[0][6],
             robot.get_results[1][4], robot.get_results[1][5], robot.get_results[1][6],
             robot.get_results[2][4], robot.get_results[2][5], robot.get_results[2][6])
    client.send(p)


def move_robot():
    # sleep(0.01)
    robot.sendSteps()
    send_params()


# Wait for incoming data from server
# .decode is used to turn the message in bytes to a string
def receive_data():
    while True:
        try:
            print("receive")
            data = client.recv(36)
            print("done")
            unp = unpack('@ffiffiffi', data)
            LS = list(unp)
            print(LS)
            robot.set(0, LS[0], LS[1], LS[2])
            robot.set(1, LS[3], LS[4], LS[5])
            robot.set(2, LS[6], LS[7], LS[8])
            # robot.set_robot_params(LS[0], LS[1], LS[2],
            #                        LS[3], LS[4], LS[5],
            #                        LS[6], LS[7], LS[8])
            move_robot()
        except ConnectionResetError:
            print("Robot has been disconnected from the server")
            break


setupGPIO()

for i in range(3):
        robot.get()

# Create new thread to wait for data
receiveThread = threading.Thread(target=receive_data)
receiveThread.start()

send_params()

# Send data to server
# str.encode is used to turn the string message into bytes so it can be sent across the network
# while True:
#     send_params()
#     sleep(1)
