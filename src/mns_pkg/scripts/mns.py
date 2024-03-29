#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)


import rospy
import numpy as np
import Constants as c
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import sys
import signal

# 조이스틱 입력을 받아서 저장하기 위한 2차원 리스트 변수 초기화
joy = [[0,0,0,0,0,0,0,0,0,0,0], 
        [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]

# node 내부의 시간의 흐름을 구현하기 위한 값 선언
turntime = Float32()
turntime.data = 0

# 관련 파라미터 선언 및 초기화
robot_mode = 9              # 알고리즘 모드를 설정하는 변수 
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
B0 = 4.36     
              # if B0=1, B is 5.5
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
    global robot_mode, click_flag

    if data[0][8]==1:       # back 버튼을 누르면 mode 9 (중립 모드)
        robot_mode = 9

    elif data[0][9]==1:    # start 버튼을 누르면: mode 0 (통신모드)
        robot_mode=0

    elif data[0][4]==1:    # LB 버튼을 누르면: mode 1 (Gradient 모드)
        robot_mode=1      
    
    elif data[0][5]==1:    # RB 버튼을 누르면: mode 2 (Helical 모드)
        robot_mode=2       
    
    elif data[0][3]==1:    # X 버튼을 누르면: mode 3 (프로그램 종료)
        robot_mode=3        

    get_theta(data)
    get_phi(data)
    get_speed(data)


# 왼쪽 조이스틱 위/아래 조작: cylindrical coordi. 기준 helical robot의 theta값 조정
def get_theta(data):
    global theta, theta_flag, theta_init

    if (data[1][1]<-0.5333) and (theta_flag==0):  # 왼쪽 조이스틱 아래로 내리면
        theta = theta_init + theta + np.pi / 12   # theta값 증가
        theta_flag = 1
    
    elif (-0.4666<=data[1][1]) and (data[1][1]<0.4666):  
        theta_flag = 0
    
    elif (0.5333<=data[1][1]) and (theta_flag==0):  # 왼쪽 조이스틱 위로 올리면
        theta = theta_init + theta - np.pi / 12     # theta값 감소
        theta_flag = 1


# 오른쪽 조이스틱 좌/우 조작: cylindrical coordi. 기준 helical robot의 phi값 조정
def get_phi(data):
    global phi, phi_flag

    if (data[1][2] < -0.6333) and (phi_flag == 0):    # 오른쪽 조이스틱 오른쪽으로 당기면
        phi = phi + np.pi / 12                        # phi값 증가
        phi_flag = 1
    
    elif (-0.4666 <= data[1][2]) and (data[1][2] < 0.4666):
        phi_flag = 0
    
    elif (0.6333 <= data[1][2]) and (phi_flag == 0):  # 오른쪽 조이스틱 윈쪽으로 당기면
        phi = phi - np.pi / 12;                       # phi값 감소
        phi_flag = 1


# 오른쪽 조이스틱 위/아래 조작: helical robot의 회전 속도
def get_speed(data):
    global speed, speed_flag

    if (data[1][3] < -0.6333) and (phi_flag == 0):    # 오른쪽 조이스틱 아래로 내리면
        speed -= 1                                    # 속도 감소
        speed_flag = 1
    
    elif (-0.4666 <= data[1][3]) and (data[1][3] < 0.4666):
        speed_flag = 0
    
    elif (0.6333 <= data[1][3]) and (phi_flag == 0):   # 오른쪽 조이스틱 위로 올리면
        speed += 1                                     # 속도 증가
        speed_flag = 1 
     

### helical robot을 회전시키기 위한 자기장 값 계산 알고리즘 ###
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

    delta = np.arctan2(np.real(N_length) , np.real(U_length))

    # [ calculate Helical B ]
    Brt = np.array(B0*(np.cos(delta)*N + np.sin(delta)*np.cos(omega*turntime.data)*U + np.sin(delta)*np.sin(omega*turntime.data)*(np.cross(N,U))))
    H = np.cross(Brt,N);    # value for drawing graph


    # get coil's magneticfield
    c.Bh = Brt[0]
    c.Buy = Brt[1]
    c.Buz = Brt[2]





### 해당 프로그램의 메인문 ###
def mns():
    global robot_mode, joy, theta, phi, speed, theta_init_flag, exit

    rospy.init_node('mns', anonymous=True)
    
    #### 메세지 구독 구간 ####
    rospy.Subscriber('/joy', Joy, joycallback)    # 조이스틱 입력 값을 가져 옴

    #### 메세지 발행 구간 ####
    pub = rospy.Publisher('mns_msgs', String, queue_size=10)  # 계산된 전압 값을 발행함
    pub_volt = rospy.Publisher('volt_array', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(100) # 100hz



    #### 본격적인 알고리즘 반복문 ####
    while (not rospy.is_shutdown()) or (exit == 1):
        

        mode_select(joy)
        # print('[%d]  [%d]  [%d]  [%d]  [%d]   { theta:%d } { phi:%d }  [%d] \n' %(np.around(c.Vh),np.around(c.Vuy),np.around(c.Vuz),np.around(c.Vm),np.around(c.Vg),np.around(np.rad2deg(theta)),np.around(np.rad2deg(phi)),speed))

        #### btn: start, mode 0: PSU 무선통신 시작 ####
        if robot_mode == 0:
            pub.publish("Comm_Start")               # 통신 시작 설정을 알리는 문자열 메세지 전송
            robot_mode = 9                          # 이후 중립모드로 설정

        #### btn: RB, mode 2: Helical Mode ####
        elif robot_mode == 2:
            helical_control()                       # 자기로봇 제어를 위한 자기장 값 계산
            print('****** Parameters Check *****\n')
            # print('HC: [%d]  \nUy: [%d]  \nUz[%d]  \nMC[%d]  \nG[%d]   \n{ theta:%d } \n{ phi:%d }  \n[%d]\n', np.around(c.Vh),np.around(c.Vuy),np.around(c.Vuz),np.around(c.Vm),np.around(c.Vg),np.around(np.rad2deg(theta)),np.around(np.rad2deg(phi)),speed)        
            print(c.volt())
            
            v_new = Float32MultiArray()
            v_new.data = c.volt()
            pub.publish("Control_Start")            # 제어 시작 설정을 알리는 문자열 메세지 전송
            pub_volt.publish(v_new)                 # publish

        #### btn: X, mode 3: 프로그램 종료 ####
        elif robot_mode == 3:   
            c.Bh = 0; c.Buy = 0; c.Buz = 0
            c.B_m = 0; c.g_m = 0; c.B_g = 0; c.g_g = 0
            theta_init_flag = 0

            pub.publish('Comm_End')                    # 통신 종료 설정을 알리는 문자열 메세지 전송 
            robot_mode = 9                             # 이후 중립모드로
            exit = 0                                   # 반복문 탈출 및 node 종료


        
        turntime.data += 0.01   # 시간의 흐름 설정 (물론 실시간 보장 X)

        rate.sleep()



    rospy.spin()
    


if __name__ == '__main__':
    mns()
