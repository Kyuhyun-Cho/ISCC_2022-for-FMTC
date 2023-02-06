#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 라바콘 위치 정보를 기반으로 생성한 차량 추종점에 보간법을 적용하여 Local Path를 생성하는 노드  ################################### 

from threading import local
import numpy as np
from std_msgs.msg import Float64,Int16,Float32MultiArray
import rospy
import rospkg
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from waypoint_maker.msg import Waypoint

# Local Path Waypoint 개수
linspace_num = 200

class drawing_path():
    def __init__(self):
        global linspace_num

        rospy.init_node('linspace')
        pub1 = rospy.Publisher('waypoint_linspace', MarkerArray, queue_size = 1)
        pub2 = rospy.Publisher('local_path', Waypoint, queue_size = 1)
        rospy.Subscriber('/waypoint_info', Waypoint, self.callback)
        self.x_array = []
        self.y_array = []
        array = [[] for _ in range(linspace_num)]

        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            
            local_path = Waypoint()

            if len(self.x_array) > 1 and len(self.y_array) > 1:

                x = self.x_array
                y = self.y_array
                cnt = self.cnt - 1

                # Waypoint X좌표와 Y좌표를 활용하여 하나의 2차 함수 생성
                try:
                    z = np.polyfit(x, y, 2) 
                except:
                    pass
                p = np.poly1d(z)
                 
                # evaluate on new data points
                x_new = np.linspace(min(x), max(x), 200) 
                y_new = p(x_new)

                for i in range(linspace_num):
                    array[i] = [x_new[i], y_new[i]]
                

                for i in range(linspace_num):
                    local_path.x_arr[i] = array[i][0]
                    local_path.y_arr[i] = array[i][1]
            
                pub2.publish(local_path)

            rate.sleep()


    def callback(self, msg):
        self.x_array = list(msg.x_arr)[0:msg.cnt]
        self.y_array = list(msg.y_arr)[0:msg.cnt]
        self.cnt = msg.cnt
        self.x_array.insert(0, -1.1)
        self.y_array.insert(0, 0)


if __name__ == '__main__':
    try:
        drawing = drawing_path()
    except rospy.ROSInterruptException:
        pass