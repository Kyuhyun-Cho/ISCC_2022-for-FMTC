#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 트랙 주행 중 차량 전방에 장애물 검출 시 모터  감속값을 발행해주는 노드 ################################### 

import rospy
import math
from object_detector.msg import ObjectInfo
from track_race.msg import Velocity
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class DynamicVelocity:
    def __init__(self):
        # dynamicVelocity attribute
        self.objectTotal = 0
        # object info array
        self.x = [0.0 for _ in range(30)]
        self.y = [0.0 for _ in range(30)]
        self.r = [0.0 for _ in range(30)]
        self.range = 1.16  # 탐색 폭 1.16 = 차량 폭 길이
        self.velocity = 10
        # scan Range
        self.minYRange = -self.range / 2
        self.maxYRange = self.range / 2

        self.a = 999
        self.b = 999
        self.c = 999
        self.d = 999
        self.yPivot = 0  # 직선이 지나는 y 좌표

        # subscriber
        rospy.Subscriber("/object_info", ObjectInfo,
                         self.mainCallBack)

        # publisher
        self.velocityPub = rospy.Publisher(
            "/dynamic_velocity_lidar", Velocity, queue_size=1)
        self.visualizeScanLinePub = rospy.Publisher(
            "/dynamic_velocity_scan_line", Marker, queue_size=1)


    def mainCallBack(self, msg):
        self.setObjectInfoArray(msg)
        self.calcVelocity()
        self.publishVelocity()

    def setObjectInfoArray(self, msg):
        self.objectTotal = msg.objectCounts
        for i in range(self.objectTotal):
            self.x[i] = msg.centerX[i]
            self.y[i] = msg.centerY[i]
            self.r[i] = msg.lengthY[i]

    def calcDistance(self, x, y):
        return math.sqrt(x*x + y*y)

    def isInRange(self, y, r):
        if (self.minYRange < (y + r/2)) and ((y - r/2) < self.maxYRange):
            return True
        return False

    def calcVelocity(self):
        distance = 999
        tmpDistance = 0

        for i in range(self.objectTotal):
            if self.isInRange(self.y[i], self.r[i]):
                tmpDistance = self.calcDistance(self.x[i], self.y[i])
                if tmpDistance < distance:
                    distance = tmpDistance
        
        print(distance)
        velocity = math.log(distance + math.e) * 4.0

        self.setVelocity(velocity)

    def setVelocity(self, velocity):
        if velocity < 8:
            self.velocity = 8
        elif 12 < velocity:
            self.velocity = 12
        else:
            self.velocity = velocity

    def getVelocity(self):
        return self.velocity

    def publishVelocity(self):
        velocityMsg = Velocity()
        velocityMsg.velocity = self.velocity
        self.velocityPub.publish(velocityMsg)

    def scanRange(self, x, y, r):
        a, b, c, d = self.a, self.b, self.c, self.d
        # 점 직선 사이 거리 계산 공식 <= 반지름 or 두 직선사이 범위
        if (abs(a*x - y + b) / math.sqrt(a*a + 1)) <= r or (abs(c*x - y + d) / math.sqrt(c*c + 1)) <= r or (a*x + b <= y <= c*x + d):
            return True
        else:
            return False

    # 차량 전방 탐색 폭 Visualization 함수
    def visualizeScanLine(self, rad):
        scanLineMsg = Marker()
        scanLineMsg.header.frame_id = "/velodyne"
        scanLineMsg.ns = "scan_line"
        scanLineMsg.id = 0

        scanLineMsg.type = Marker.LINE_LIST
        scanLineMsg.action = Marker.ADD

        scanLineMsg.color = ColorRGBA(1, 0, 0, 1)
        scanLineMsg.scale.x = 0.1

        scanLineMsg.points.append(Point(self.yPivot, -self.range/2, 0))
        scanLineMsg.points.append(
            Point(math.cos(rad) * 10, -self.range/2 - math.sin(rad) * 10, 0))
        scanLineMsg.points.append(Point(self.yPivot, self.range/2, 0))
        scanLineMsg.points.append(
            Point(math.cos(rad) * 10, self.range/2 - math.sin(rad) * 10, 0))

        self.visualizeScanLinePub.publish(scanLineMsg)

if __name__ == '__main__':
    rospy.init_node("dynamic_velocity_node")
    dynamicVelocity = DynamicVelocity()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
