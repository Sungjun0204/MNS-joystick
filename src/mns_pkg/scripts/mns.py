#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import numpy as np
import Constants as c
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import sys
import signal


joy = [[0,0,0,0,0,0,0,0,0,0,0], 
        [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]

turntime = Int32()
turntime.data = 0
robot_mode = 0
theta=0
thetafix=0
theta_flag=0
phi=0
phifix=0
phi_flag=0
speed=0
speed_flag=0
exit=0
theta_init = 0
theta_init_flag = 0

MU0 = 4*np.pi*17**(-7)
B0 = 3.64     # if B0=1, B is 5.5
              # hanyang Uni. spec standard: 2.55(14mT)
              # Kwangwoon Uni. spec standard: 3.64(20mT)





def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)






def joycallback(data):
    global joy
    joy[0] = data.buttons
    joy[1] = data.axes


def mode_select(data):
    global robot_mode
    if data[0][7]==1:    # if press 'start' button
        robot_mode=1       # gradient mode
    
    elif data[0][6]==1:  # if press 'back' button
        robot_mode=2       # helical mode
    
    elif data[0][2]==1:  # if press 'X' button
        robot_mode=3        # code end

    get_theta(data)
    get_phi(data)
    get_speed(data)


def get_theta(data):
    global theta, theta_flag, theta_init

    if (data[1][1]<-0.5333) and (theta_flag==0):
        theta = theta_init + theta + np.pi / 12
        theta_flag = 1
    
    elif (-0.4666<=data[1][1]) and (data[1][1]<0.4666):
        theta_flag = 0
    
    elif (0.5333<=data[1][1]) and (theta_flag==0):
        theta = theta_init + theta - np.pi / 12  
        theta_flag = 1


def get_phi(data):
    global phi, phi_flag

    if (data[1][4] < -0.6333) and (phi_flag == 0):
        phi = phi + np.pi / 12
        phi_flag = 1
    
    elif (-0.4666 <= data[1][4]) and (data[1][4] < 0.4666):
        phi_flag = 0
    
    elif (0.6333 <= data[1][4]) and (phi_flag == 0):
        phi = phi - np.pi / 12;   
        phi_flag = 1


def get_speed(data):
    global speed, speed_flag

    if (data[1][3] < -0.6333) and (phi_flag == 0):
        speed -= 1
        speed_flag = 1
    
    elif (-0.4666 <= data[1][3]) and (data[1][3] < 0.4666):
        speed_flag = 0
    
    elif (0.6333 <= data[1][3]) and (phi_flag == 0):
        speed += 1
        speed_flag = 1 
     

def helical_control():
    global theta_init_flag, theta_init, speed, theta

    if (theta_init_flag == 0):
        theta_init = 1.57  # 90 deg reste

    if (theta_flag != 9.469697e-04) and (theta_init_flag == 0):
        theta_init = 0
        get_theta(joy)
        theta_init_flag = 1

    omega = speed * 1 * 2 * np.pi
    f = omega / (2 * np.pi)
    print('Hz: [%d]\n' %(np.around(f)))


    # basic equations in paper
    U = np.array([np.sin(theta), -np.cos(theta)*np.cos(phi), -np.cos(theta)*np.sin(phi)])
    N = np.array([np.cos(theta), np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi)])

    # calculate each delta
    U_length = np.sqrt(((U[0]**2)-(N[0]**2)) + ((U[1]**2)-(N[1]**2)) + ((U[2]**2)-(N[2]**2)))
    N_length = np.sqrt((N[0]**2) + (N[1]**2) + (N[2]**2))

    # delta = atan(N_length / U_length);
    delta = np.arctan2(np.real(N_length) , np.real(U_length))

    # [ calculate Helical B ]
    # Brt = [[0.0], [0.0], [0.0]]
    Brt = np.array(B0*(np.cos(delta)*N + np.sin(delta)*np.cos(omega*turntime.data)*U + np.sin(delta)*np.sin(omega*turntime.data)*(np.cross(N,U))))
    H = np.cross(Brt,N);    # value for drawing graph


    # get coil's magneticfield
    c.Bh = Brt[0]
    c.Buy = Brt[1]
    c.Buz = Brt[2]
    # print(c.Bh, c.Buy, c.Buz)






def mns():
    global robot_mode, joy, theta, phi, speed, theta_init_flag, exit

    rospy.init_node('mns', anonymous=True)

    #### Subscribe Section ####
    rospy.Subscriber('/joy', Joy, joycallback)

    #### Publish Section ####
    pub = rospy.Publisher('mns_msgs', String, queue_size=10)
    rate = rospy.Rate(100) # 100hz



    # repeating
    while (not rospy.is_shutdown()) or (exit == 1):
        
        mode_select(joy)
        # print('[%d]  [%d]  [%d]  [%d]  [%d]   { theta:%d } { phi:%d }  [%d] \n' %(np.around(c.Vh),np.around(c.Vuy),np.around(c.Vuz),np.around(c.Vm),np.around(c.Vg),np.around(np.rad2deg(theta)),np.around(np.rad2deg(phi)),speed))

        # DC_8ports_setting
        if robot_mode == 0:
            rospy.loginfo("PSU is connected now!")
            # PSU_set = ['SYST:REM', 'OUTP 1', 'MODE DC', 'INSTrument:COUPle NONE']
            pub.publish('SYST:REM')
            pub.publish('OUTP 1')
            pub.publish('MODE DC')
            pub.publish('INSTrument:COUPle NONE')
            robot_mode = 9

        
        elif robot_mode == 2: 
            helical_control()
            print('****** Parameters Check *****\n')
            print('HC: [%d]  \nUy: [%d]  \nUz[%d]  \nMC[%d]  \nG[%d]   \n{ theta:%d } \n{ phi:%d }  \n[%d]\n', np.around(c.Vh),np.around(c.Vuy),np.around(c.Vuz),np.around(c.Vm),np.around(c.Vg),np.around(np.rad2deg(theta)),np.around(np.rad2deg(phi)),speed)        
            print(c.Vh, c.Vuy, c.Vuz)

        elif robot_mode == 3:
            c.Bh = 0; c.Buy = 0; c.Buz = 0
            c.B_m = 0; c.g_m = 0; c.B_g = 0; c.g_g = 0
            theta_init_flag = 0
            
            rospy.loginfo("PSU is unconnected!!!!!!")
            # PSU_off = ['INSTrument:COUPle ALL', ''.join(['VOLTage',chr(0)]), 'OUTP 0', 'SYST:LOC']
            pub.publish('INSTrument:COUPle ALL')
            pub.publish(''.join(['VOLTage',chr(0)]))
            pub.publish('OUTP 0')
            pub.publish('SYST:LO')
            robot_mode = 9
            exit = 1

        # Transmit DC
        v_new = [0.0, 0.0, 0.0]
        v_new[0] = c.Vh; v_new[1] = c.Vuy; v_new[2] = c.Vuz
        pub.publish(''.join(['INSTrument:NSELect',chr(1)]))  # phase 1
        pub.publish(''.join(['VOLTage',str(v_new[0])]))

        pub.publish(''.join(['INSTrument:NSELect',chr(2)]))  # phase 2
        pub.publish(''.join(['VOLTage',str(v_new[1])]))

        pub.publish(''.join(['INSTrument:NSELect',chr(3)]))  # phase 3
        pub.publish(''.join(['VOLTage',str(v_new[2])]))
        
        
        turntime.data += 1

        rate.sleep()


    


    rospy.spin()
    


if __name__ == '__main__':
    mns()
