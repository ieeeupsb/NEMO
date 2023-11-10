import time
import pygame
import threading
import zmq

import sys
sys.path.append("nodes_src/")
from zmq_nemo import *


# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

context = zmq.Context()
v_w_publisher = create_publisher(context, "v_w_commands")
eletroiman_publisher = create_publisher(context, "em_commands")

# Define a function to send a command to the Raspberry Pi Pico

while True:
    # Get the joystick input and calculate the v parameter
    pygame.event.pump()
    # use axis 1 for vertical movement
    joystick_axis_v = joystick.get_axis(1)
    joystick_axis_w = joystick.get_axis(3)

    if joystick.get_button(0):
        print('X - Ligar eletroiman')
        send_em(eletroiman_publisher, 255)

    elif joystick.get_button(1):
        print('O - Desligar eletroiman')
        send_em(eletroiman_publisher, 0)

    # map joystick range (-1, 1) to command range (-5000, 5000)
    # v = int(round(joystick_axis, 1) * -0.7)
    # w = int(round(joystick_axis2, 1) * -0.3)
    v = round(-joystick_axis_v, 1) * 0.8
    w = round(-joystick_axis_w, 1) * 3
    print("v: " + str(v))
    print("w: " + str(w))
    # Store the v parameter in a global variable
    send_v_w(v_w_publisher, v, w)

    # add a small delay to prevent sending commands too quickly
    time.sleep(0.1)