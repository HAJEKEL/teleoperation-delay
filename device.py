import pygame
import numpy as np
import math
import matplotlib.pyplot as plt
from pantograph import Pantograph
from pyhapi import Board, Device, Mechanisms
from pshape import PShape
import sys, serial, glob
from serial.tools import list_ports
import time
import socket, struct
import queue


UDP_IP = "127.0.0.1"
UDP_PORT_IN = 40001
UDP_PORT_OUT = 40002

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create a send socket
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create a receive socket
recv_sock.bind((UDP_IP, UDP_PORT_IN))  # bind the scoket to port 40002
print('fill socket')

position = np.array([0,0])
send_data = bytearray(struct.pack("=%sf" % position.size, *position))  # convert array of 2 floats to bytes
send_sock.sendto(send_data, (UDP_IP, UDP_PORT_OUT))  # send to IP address UDP_IP and port UDP_PORT_OUT

##################### General Pygame Init #####################
##initialize pygame window
pygame.init()
window = pygame.display.set_mode((600, 400))   ##twice 600x400 for haptic and VR
pygame.display.set_caption('Virtual Haptic Device')

screenHaptics = pygame.Surface((600,400))

##add nice icon from https://www.flaticon.com/authors/vectors-market
icon = pygame.image.load('robot.png')
pygame.display.set_icon(icon)

##add text on top to debugToggle the timing and forces
font = pygame.font.Font('freesansbold.ttf', 18)

pygame.mouse.set_visible(True)  ##Hide cursor by default. 'm' toggles it

##set up the on-screen debugToggle
text = font.render('Virtual Haptic Device', True, (0, 0, 0), (255, 255, 255))
textRect = text.get_rect()
textRect.topleft = (10, 10)

##initialize "real-time" clock
clock = pygame.time.Clock()
FPS = 100   #in Hertz

##define some colors
cWhite = (255,255,255)
cDarkblue = (36,90,190)
cLightblue = (0,176,240)
cRed = (255,0,0)
cOrange = (255,100,0)
cYellow = (255,255,0)

####Pseudo-haptics dynamic parameters, k/b needs to be <1
k = .5       ##Stiffness between cursor and haptic display
b = .8       ##Viscous of the pseudohaptic display
dt = 0.01

##################### Define sprites #####################

##define sprites
hhandle = pygame.image.load('handle.png')
haptic  = pygame.Rect(*screenHaptics.get_rect().center, 0, 0).inflate(48, 48)
cursor  = pygame.Rect(0, 0, 5, 5)

buffer = 1 #length of buffer Delay. For 1 is no buffer

xh = np.array(haptic.center)
q = queue.Queue(buffer)

##Set the old value to 0 to avoid jumps at init
xhold = 0
xmold = 0


##################### Init Virtual env. #####################


##################### Detect and Connect Physical device #####################
# USB serial microcontroller program id data:
def serial_ports():
    """ Lists serial port names """
    ports = list(serial.tools.list_ports.comports())

    result = []
    for p in ports:
        try:
            port = p.device
            s = serial.Serial(port)
            s.close()
            if p.description[0:12] == "Arduino Zero":
                result.append(port)
                print(p.description[0:12])
        except (OSError, serial.SerialException):
            pass
    return result


CW = 0
CCW = 1

haplyBoard = Board
device = Device
SimpleActuatorMech = Mechanisms
pantograph = Pantograph
robot = PShape

#########Open the connection with the arduino board#########
port = serial_ports()  ##port contains the communication port or False if no device
if port:
    print("Board found on port %s" % port[0])
    haplyBoard = Board("test", port[0], 0)
    device = Device(5, haplyBoard)
    pantograph = Pantograph()
    device.set_mechanism(pantograph)

    device.add_actuator(1, CCW, 2)
    device.add_actuator(2, CW, 1)

    device.add_encoder(1, CCW, 241, 10752, 2)
    device.add_encoder(2, CW, -61, 10752, 1)

    device.device_set_parameters()
else:
    print("No compatible device found. Running virtual environnement...")
    # sys.exit(1)

# conversion from meters to pixels
window_scale = 3

##################### Main Loop #####################
##Run the main loop

run = True
ongoingCollision = False
fieldToggle = True
robotToggle = True
debugToggle = False
delay = True

#variables
fe = np.zeros(2)

print('starting loop')
while run:

    #########Process events  (Mouse, Keyboard etc...)#########
    for event in pygame.event.get():
        ##If the window is close then quit
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('m'):  ##Change the visibility of the mouse
                pygame.mouse.set_visible(not pygame.mouse.get_visible())
            if event.key == ord('q'):  ##Force to quit
                run = False
            if event.key == ord('d'):
                debugToggle = not debugToggle
            if event.key == ord('r'):
                robotToggle = not robotToggle
            if event.key == ord('m'):
                delay = not delay


    ######### Read position (Haply and/or Mouse)  #########

    ##Get endpoint position xh
    if port and haplyBoard.data_available():  ##If Haply is present
        # Waiting for the device to be available
        #########Read the motorangles from the board#########
        device.device_read_data()
        motorAngle = device.get_device_angles()

        #########Convert it into position#########
        device_position = device.get_device_position(motorAngle)
        xh = np.array(device_position) * 1e3 * window_scale
        xh[0] = np.round(-xh[0] + 300)
        xh[1] = np.round(xh[1] - 60)
        xm = xh  ##Mouse position is not used
        recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
        force = struct.unpack("2f",
                              recv_data)  # convert the received data from bytes to array of 3 floats (assuming force in 3 axes)



    else:
        ##Compute distances and forces between blocks
        xh = np.clip(np.array(haptic.center), 0, 599)
        xh = np.round(xh)

        ##Get mouse position
        cursor.center = pygame.mouse.get_pos()
        xm = np.clip(np.array(cursor.center), 0, 599)

        recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
        force = struct.unpack("2f",recv_data)  # convert the received data from bytes to array of 3 floats (assuming force in 3 axes)

    ######### Send forces to the device #########
    if port:
        fe[1] = -fe[1]  ##Flips the force on the Y=axis

        ##Update the forces of the device
        device.set_device_torques(fe)
        device.device_write_torques()
        # pause for 1 millisecond
        time.sleep(0.001)
    else:
        ######### Update the positions according to the forces ########
        ##Compute simulation (here there is no inertia)
        ##If the haply is connected xm=xh and dxh = 0
        dxh = (k / b * (xm - xh) / window_scale - fe / b)  ####replace with the valid expression that takes all the forces into account
        dxh = dxh * window_scale
        xh = np.round(xh + dxh)  ##update new positon of the end effector

    haptic.center = xh

    ##Update old samples for velocity computation
    xhold = xh
    xmold = xm

    ######### Graphical output #########
    ##Render the haptic surface
    screenHaptics.fill(cWhite)

    ##Change color based on effort
    colorMaster = (255,
                   255 - np.clip(np.linalg.norm(k * (xm - xh) / window_scale) * 15, 0, 255),
                   255 - np.clip(np.linalg.norm(k * (xm - xh) / window_scale) * 15, 0,
                                 255))  # if collide else (255, 255, 255)

    pygame.draw.rect(screenHaptics, colorMaster, haptic, border_radius=8)

    ######### Robot visualization ###################
    # update individual link position
    if robotToggle:
        robot.createPantograph(screenHaptics, xh)

    ### Hand visualisation
    screenHaptics.blit(hhandle, (haptic.topleft[0], haptic.topleft[1]))
    pygame.draw.line(screenHaptics, (0, 0, 0), (haptic.center), (haptic.center + 2 * k * (xm - xh)))

    ##Fuse it back together
    window.blit(screenHaptics, (0,0))
    ##Print status in  overlay
    if debugToggle:
        text = font.render("FPS = " + str(round(clock.get_fps())) +
                           "  xm = " + str(np.round(10 * xm) / 10) +
                           "  xh = " + str(np.round(10 * xh) / 10) +
                           "  fe = " + str(np.round(10 * fe) / 10)
                           , True, (0, 0, 0), (255, 255, 255))
        window.blit(text, textRect)

    pygame.display.flip()

    pos = np.array([4  * xh[0]/ 3, 3 * xh[1] / 2])
    if delay:
        q.put(pos)
        if q.full() == True:
            position = q.get()
        else:
            position = np.array([200, 200])

        send_data = bytearray(struct.pack("=%sf" % position.size, *position))  # convert array of 2 floats to bytes
        send_sock.sendto(send_data, (UDP_IP, UDP_PORT_OUT))  # send to IP address UDP_IP and port UDP_PORT_OUT
    else:
        send_data = bytearray(struct.pack("=%sf" % pos.size, *pos))  # convert array of 2 floats to bytes
        send_sock.sendto(send_data, (UDP_IP, UDP_PORT_OUT))  # send to IP address UDP_IP and port UDP_PORT_OUT

    ##Slow down the loop to match FPS
    clock.tick(FPS)

pygame.display.quit()
pygame.quit()
