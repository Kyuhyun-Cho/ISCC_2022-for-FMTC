#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 트랙 주행 전용 모터 컨트롤 노드 ################################### 

from queue import Queue
from time import time
import rospy
import sys
from std_msgs.msg import Bool
from morai_msgs.msg import CtrlCmd
from track_race.msg import Velocity
from track_race.msg import Steering
from std_msgs.msg import Float64
from track_race.msg import Test
from race.msg import drive_values
import erp_info


class control:
    def __init__(self, flag = 1):
        self.mode = flag  # 0: morai_simulation, 1: real_epr42
        self.oneLapFlag = False
        self.steeringLidar = 0.0
        self.velocityLidar = 0.0
        self.steeringGps = 0.0
        self.velocityGps = 0.0
        self.erp42Car = erp_info.erp42()
        self.erp42CarMorai = erp_info.erp42morai()
        self.currentSpeed = 0
        self.coolTime = 3.0
        self.currentTime = time()
        self.brakeCoolTime = time()

        # Subscriber
        rospy.Subscriber("/is_one_lap_finished", Bool, self.oneLapFlagCallback) # 선 GPS 주행 후 라이다 센서 기반 주행으로 변경하기 위해 구독 하는 토픽

        rospy.Subscriber("/dynamic_velocity_lidar", Velocity, self.velocityLidarCallback) # 전방 일정 거리 이내 장애물 감지 시 적절한 감속을 위한 속도 발행 토픽_라이다 주행 전용
        rospy.Subscriber("/pure_pursuit_lidar", Steering, self.steeringLidarCallback)      # 속도에 따른 적절한 조향을 위한 조향각 발행 토픽_라이다 주행 전용

        rospy.Subscriber("/dynamic_velocity_gps", Velocity, self.velocityGpsCallback) # 전방 일정 거리 이내 장애물 감지 시 적절한 감속을 위한 속도 발행 토픽_GPS 주행 전용
        rospy.Subscriber("/pure_pursuit_gps", Steering, self.steeringGpsCallback)      # 속도에 따른 적절한 조향을 위한 조향각 발행 토픽_GPS 주행 전용

        rospy.Subscriber("/gps_velocity", Float64, self.getCurrentSpeed) # GPS 센서를 기반으로 계산한 차량의 현재 속도

        # Publisher
        self.erp42ControlPublisher = self.getPublisher(self.mode)
        self.test_steeringDeltaPublisher = rospy.Publisher("/delta", Test, queue_size=1)


    def oneLapFlagCallback(self, msg):
        self.oneLapFlag = msg.data

    def velocityLidarCallback(self, msg):
        self.velocityLidar = msg.velocity

    def steeringLidarCallback(self, msg):
        self.steeringLidar = msg.steering

    def velocityGpsCallback(self, msg):
        self.velocityGps = msg.velocity

    def steeringGpsCallback(self, msg):
        self.steeringGps = msg.steering

    def getPublisher(self, mode):
        if self.mode == 0:
            return rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        else:
            return rospy.Publisher("/control_value", drive_values, queue_size=1)

    def getCurrentSpeed(self, msg):
        self.currentSpeed = msg.data

    def gapThrottleAndCurrentSpeed(self):
        if self.oneLapFlag:
            return self.velocityGps - self.currentSpeed
        else:
            return self.velocityLidar - self.currentSpeed
    
    def decelerate(self):
        if self.oneLapFlag:
            ctrlMsg = drive_values()
            ctrlMsg.throttle = 0
            ctrlMsg.steering = self.steeringGps
            ctrlMsg.brake = 0.1 
        else:
            ctrlMsg = drive_values()
            ctrlMsg.throttle = 0
            ctrlMsg.steering = self.steeringLidar
            ctrlMsg.brake = 0.1 

        self.erp42ControlPublisher.publish(ctrlMsg)

    def accelerate(self):
        if self.oneLapFlag:
            ctrlMsg = drive_values()
            ctrlMsg.throttle = 20
            ctrlMsg.steering = self.steeringGps
            ctrlMsg.brake = 0.0
        else:
            ctrlMsg = drive_values()
            ctrlMsg.throttle = 20
            ctrlMsg.steering = self.steeringLidar
            ctrlMsg.brake = 0.0

        self.erp42ControlPublisher.publish(ctrlMsg)

    def isNotBrakeCoolTime(self):
        if (time() - self.brakeCoolTime) > self.coolTime:
            return True
        else:
            return False

    def controlERP42(self):
        if ( self.currentSpeed > 10 and abs(self.steeringGps) > 3 ) or (self.gapThrottleAndCurrentSpeed < -3):
            self.decelerate()
            return


        if self.oneLapFlag == False:
            if self.mode == 0:
                ctrlMsg = CtrlCmd()
                ctrlMsg.longlCmdType = 2
                ctrlMsg.velocity = self.velocityLidar + self.gapThrottleAndCurrentSpeed()
                ctrlMsg.steering = self.steeringLidar

                self.erp42ControlPublisher.publish(ctrlMsg)

            elif self.mode == 1:
                ctrlMsg = drive_values()
                ctrlMsg.throttle = self.velocityLidar + self.gapThrottleAndCurrentSpeed()
                ctrlMsg.steering = self.steeringLidar
                ctrlMsg.brake = 0

                self.erp42ControlPublisher.publish(ctrlMsg)

        elif self.oneLapFlag == True:
            if self.mode == 0:
                ctrlMsg = CtrlCmd()
                ctrlMsg.longlCmdType = 2
                ctrlMsg.velocity = self.velocityGps + self.gapThrottleAndCurrentSpeed()
                ctrlMsg.steering = self.steeringGps

                self.erp42ControlPublisher.publish(ctrlMsg)

            elif self.mode == 1:
                ctrlMsg = drive_values()
                ctrlMsg.throttle = self.velocityGps + self.gapThrottleAndCurrentSpeed()
                ctrlMsg.steering = self.steeringGps
                ctrlMsg.brake = 0

                self.erp42ControlPublisher.publish(ctrlMsg)
        

    # 현재 미사용

    def test_steeringDeltaPub(self):
        # print("test")
        testMsg = Test()
        testMsg.delta = self.delta
        self.test_steeringDeltaPublisher.publish(testMsg)


if __name__ == '__main__':
    rospy.init_node("controller_node")
    flag = int(sys.argv[1])
    controller = control(flag)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        controller.controlERP42()
        rate.sleep()
