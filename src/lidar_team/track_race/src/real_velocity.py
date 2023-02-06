#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 인코더 센서를 활용한 차량의 현재 속도 계산 노드 ################################### 
 
from base64 import encode
import rospy
from race.msg import drive_values
from std_msgs.msg import Int32, Float64
import time
import math

def calcVelocity(data, t):
    timeGap = time.time() - t

def encoder_callback(msg):
    global count
    if msg.data > 10000:
        count = msg.data

def control_callback(msg):
    global control
    control = msg.throttle     

if __name__ == '__main__':
    global count, control
    dataMemory = 0
    t = time.time()
    count = 0
    flag = 0
    control = 0
    rospy.init_node("encoder_node")
    rospy.Subscriber("/encoder_value", Int32, encoder_callback)
    rospy.Subscriber("/control_value", drive_values, control_callback)
    velPub = rospy.Publisher("gps_velocity", Float64, queue_size = 1)

    rate = rospy.Rate(60)
    start = time.time()
    while not rospy.is_shutdown():
        if count != 0:
            start_count = count
            while True:
                if time.time() - start >= 0.4:

                    end_count = count
                    encoder_vel = round(1.65*2.54*6.5*2*math.pi/100/1000/100*3600/0.4*(end_count - start_count), 4) # 인코더 센서를 활용한 차량의 현재 속도 계산
                    velPub.publish(encoder_vel)
        
                    print(end_count - start_count)

                    count = 0
                    start = time.time()
                    break

        rate.sleep()
