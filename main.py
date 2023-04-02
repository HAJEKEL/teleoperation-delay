### PA3

import numpy as np
import math
import pygame
import socket, struct
import pandas as pd

######### Init UDP connection #########
## Set IP and Port
UDP_IP = "127.0.0.1"
UDP_PORT_IN = 40002
UDP_PORT_OUT = 40001

send_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a send socket
recv_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a receive socket
recv_sock.bind((UDP_IP, UDP_PORT_IN)) # bind the scoket to port 40002
print('fill socket')

position = np.array([0,0])
send_data = bytearray(struct.pack("=%sf" % position.size, *position))  # convert array of 2 floats to bytes
send_sock.sendto(send_data, (UDP_IP, UDP_PORT_OUT))  # address UDP_IP and port UDP_PORT_OUT

class robot_arm_2dof:
    def __init__(self, l):
        self.l = l  # link length

    # arm Jacobian matrix
    def Jacobian(self, q):
        J = np.array([[-self.l[0] * np.sin(q[0]) - self.l[1] * np.sin(q[0] + q[1]),
                       -self.l[1] * np.sin(q[0] + q[1])],
                      [self.l[0] * np.cos(q[0]) + self.l[1] * np.cos(q[0] + q[1]),
                       self.l[1] * np.cos(q[0] + q[1])]])
        return J

    # inverse kinematics
    def IK(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0] ** 2 + p[1] ** 2)
        q[1] = np.pi - math.acos((self.l[0] ** 2 + self.l[1] ** 2 - r ** 2) / (2 * self.l[0] * self.l[1]))
        q[0] = math.atan2(p[1], p[0]) - math.acos((self.l[0] ** 2 - self.l[1] ** 2 + r ** 2) / (2 * self.l[0] * r))

        return q

'''SIMULATION'''

# SIMULATION PARAMETERS
dt = 0.01 # intergration step timedt = 0.01 # integration step time
dts = dt*1 # desired simulation step time (NOTE: it may not be achieved)

##define some colors
cWhite = (255,255,255)
cDarkblue = (36,90,190)
cLightblue = (0,176,240)
cRed = (255,0,0)
cOrange = (255,100,0)
cYellow = (255,255,0)

# ROBOT PARAMETERS
x0 = 0.0 # base x position
y0 = 0.0 # base y position
l1 = 0.33 # link 1 length
l2 = 0.33 # link 2 length (includes hand)
l = [l1, l2] # link length

# IMPEDANCE CONTROLLER PARAMETERS
K = np.diag([300,300]) # stiffness matrix N/m
stiffness_increment = 100 # for tele-impedance

# SIMULATOR
# initialise robot model class
model = robot_arm_2dof(l)

# initialise real-time plot with pygame
pygame.init() # start pygame
window = pygame.display.set_mode((800, 600)) # create a window (size in pixels)
window.fill((255,255,255)) # white background
xc, yc = window.get_rect().center # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255)) # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dts) # refresh rate

##create rectangles for drawing the task
start = pygame.Rect(200, 288, 25, 25)
end = pygame.Rect(575, 288, 25, 25)
weld = pygame.Rect(213, 100, 375, 400)

# initial conditions
t = 0.0 # time
t2 = 0.0 #ref time
pm = np.zeros(2) # mouse position
pr = np.zeros(2) # reference endpoint position
p = np.array([0.1,0.1]) # actual endpoint position
dp = np.zeros(2) # actual endpoint velocity
F = np.zeros(2) # endpoint force
q = np.zeros(2) # joint position
p_prev = np.zeros(2) # previous endpoint position
m = 0.5 # endpoint mass
i = 0 # loop counter
state = [] # state vector
x2 = 0 # initial x position end of arm in pygame coordinates
y2 = 0 # initial y position end of arm in pygame coordinates

## Setup data variables
person_num = 0
num = 1
tot_error = 0
tot_serror = 0
tot_serror2 = 0
tot_error2 = 0

error_list = []
serror_list = []
error2_list = []
serror2_list = []
num_list = []
person_list = []
t2_list =[]

# scaling
window_scale = 800 # conversion from meters to pixles

# wait until the start button is pressed
run = True
while run:
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.KEYUP:
            if event.key == ord('e'): # enter the main loop after 'e' is pressed
                run = False

# MAIN LOOP
i = 0
run = True
test = False

while run:
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.QUIT: # force quit with closing the window
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('q'): # force quit with q button
                run = False

            # tele-impedance interface / switch controllers
            if event.key == ord('x'):
                if K[0, 0] < 1000:
                    K[0, 0] = K[0, 0] + stiffness_increment
                else:
                    K[0, 0] = 1000
            if event.key == ord('z'):
                if K[0, 0] > 0:
                    K[0, 0] = K[0, 0] - stiffness_increment
                else:
                    K[0, 0] = 0
            if event.key == ord('d'):
                if K[1, 1] < 1000:
                    K[1, 1] = K[1, 1] + stiffness_increment
                else:
                    K[1, 1] = 1000
            if event.key == ord('c'):
                if K[1, 1] > 0:
                    K[1, 1] = K[1, 1] - stiffness_increment
                else:
                    K[1, 1] = 0
            if event.key == ord('p'):   #event key for resetting test info for new test person
                person_num +=1
                num = 1
                print(person_num, num)

# main control code
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    pm = struct.unpack("2f",
                       recv_data)  # convert the received data from bytes to array of 2 floats (assuming force in 3 axes)

    # main control code
    # pm = np.array(pygame.mouse.get_pos())
    pr = [((pm[0]) / window_scale) - (800 / (2 * window_scale)), ((-pm[1]) / window_scale) + (600 / (2 * window_scale))]

    # impedance control with enlarged damping to simulate underwater movement
    xi = .7
    D = 2 * xi * np.sqrt(K)
    dp_ = (p_prev - p) / dt
    F = K @ (pr - p) + D @ (dp_)*3

    #Wave functions
    F[0] += 5*np.sin(np.pi*t)
    F[1] += 10*np.cos(0.9*np.pi * t)

    ############TEST DATA COLLECTION##################
    ##Start of the test
    if test==False and start.collidepoint(window_scale*x2+xc,-window_scale*y2+yc)==True:
        print("start test")
        it= 0
        test = True

    ##End of the test
    if test==True and end.collidepoint(window_scale*x2+xc,-window_scale*y2+yc)==True:
        print("end test", num)
        error_list.append([tot_error/it]) #List all variable for the total error in pixels
        serror_list.append([tot_serror / it]) #List all variable for the total squared error in pixels
        error2_list.append([tot_error2 / it]) #List all variable for the total error in meters
        serror2_list.append([tot_serror2 / it]) #List all variable for the total squared error in meters
        num_list.append([num]) #List Test number
        person_list.append([person_num]) #List test person
        t2_list.append([t2]) #list completion time

        num += 1
        tot_error = 0 #Reset Variable for the total error in pixels
        tot_serror = 0 #Reset Variable for the total squared error in pixels
        tot_serror2 =0 #Reset Variable for the total error in meters
        tot_error2 =0 #Reset Variable for the total squared error in meters
        t2 = 0 #Reset completion time
        test = False

    if test:
        if window_scale*x2+xc > 200 and window_scale*x2+xc < 599:
            angle = (((window_scale*x2+xc)/400)-0.5)*np.pi
            ys = -200 * np.sin(angle) + 299
            err = abs((-window_scale * y2 + yc - ys)) #Error allong the arc
            #Collect the errors
            tot_serror += err**2
            tot_error += err
            tot_serror2 += (err/window_scale) ** 2
            tot_error2 += err/window_scale
            it += 1
        else:
            # Collect the errors
            err = abs((-window_scale * y2 + yc - 299)) #error away from arc
            tot_serror += err ** 2
            tot_error += err
            tot_serror2 += (err / window_scale) ** 2
            tot_error2 += err / window_scale
            it += 1
        t2 += dt

    #prepare Force for encryption
    force = np.array(F)

	# previous endpoint position for velocity calculation
    p_prev = p.copy()

    # log states for analysis
    state.append([t, pr[0], pr[1], p[0], p[1], dp[0], dp[1], F[0], F[1], K[0,0], K[1,1]])

    # integration
    ddp = F/m
    dp += ddp*dt
    p += dp*dt
    t += dt

    # increase loop counter
    i = i + 1

    # update individual link position
    q = model.IK(p)
    x1 = l1*np.cos(q[0])
    y1 = l1*np.sin(q[0])
    x2 = x1+l2*np.cos(q[0]+q[1])
    y2 = y1+l2*np.sin(q[0]+q[1])

    # real-time plotting
    window.fill(cLightblue) # clear window

    pygame.draw.arc(window,cOrange,weld,0, np.pi, 2) #draw the trajectory line
    # pygame.draw.rect(window, cOrange, weld)
    pygame.draw.rect(window, cWhite, start) # Draw the white start square
    pygame.draw.rect(window, cRed, end) # Draw the red end square

    #pygame.draw.circle(window, (0, 255, 0), (pm[0], pm[1]), 5) # draw reference position
    pygame.draw.lines(window, (0, 0, 255), False, [(window_scale*x0+xc,-window_scale*y0+yc), (window_scale*x1+xc,-window_scale*y1+yc), (window_scale*x2+xc,-window_scale*y2+yc)], 6) # draw links
    pygame.draw.circle(window, (0, 0, 0), (window_scale*x0+xc,-window_scale*y0+yc), 9) # draw shoulder / base
    pygame.draw.circle(window, (0, 0, 0), (window_scale*x1+xc,-window_scale*y1+yc), 9) # draw elbow
    pygame.draw.circle(window, (255, 0, 0), (window_scale*x2+xc,-window_scale*y2+yc), 5) # draw hand / endpoint

    force_scale = 50/(window_scale*(l1*l1)) # scale for displaying force vector
    pygame.draw.line(window, (0, 255, 255), (window_scale*x2+xc,-window_scale*y2+yc), ((window_scale*x2+xc)+F[0]*force_scale,(-window_scale*y2+yc-F[1]*force_scale)), 2) # draw endpoint force vector

    # print data
    text = font.render(
        "FPS = " + str(round(clock.get_fps())) + "   K = " + str([K[0, 0], K[1, 1]]) + " N/m" + "   x = " + str(
            np.round(p, 3)) + " m" + "   F = " + str(np.round(F, 0)) + " N", True, (0, 0, 0), (255, 255, 255))
    window.blit(text, textRect)

    pygame.display.flip()  # update display

    # send Data by UDP
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 2 floats to bytes
    send_sock.sendto(send_data, (UDP_IP, UDP_PORT_OUT))  # send to IP address UDP_IP and port UDP_PORT_OUT

    # try to keep it real time with the desired step time
    clock.tick(FPS)

    if run == False:
        break


pygame.quit()  # stop pygame

#After ending the simulation the Test data is printed and saved in a excel file
print(person_list, num_list, t2_list, error_list, serror_list, error2_list, serror2_list)

# convert test data to panda DataFrames
df = pd.DataFrame (person_list)
df1 = pd.DataFrame (num_list)
df2 = pd.DataFrame (t2_list)
df3 = pd.DataFrame (error_list)
df4 = pd.DataFrame (serror_list)
df5 = pd.DataFrame (error2_list)
df6 = pd.DataFrame (serror2_list)

filepath = 'my_excel_file.xlsx' #Set excel filepath to where you want to save your data
sheet = 'Delay_x' #Set sheet name to where you want to save your data

#write data to excel
with pd.ExcelWriter(filepath,
    mode="a",
    engine="openpyxl",
    if_sheet_exists="overlay") as writer:
    df.to_excel(writer, sheet_name=sheet, index=False )
    df1.to_excel(writer, sheet_name=sheet, startcol=2, index=False )
    df2.to_excel(writer, sheet_name=sheet, startcol=4, index=False )
    df3.to_excel(writer, sheet_name=sheet, startcol=6, index=False )
    df4.to_excel(writer, sheet_name=sheet, startcol=8, index=False )
    df5.to_excel(writer, sheet_name=sheet, startcol=10, index=False )
    df6.to_excel(writer, sheet_name=sheet, startcol=12, index=False )
